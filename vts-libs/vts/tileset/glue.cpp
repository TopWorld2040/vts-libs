#include <boost/format.hpp>

#include "utility/progress.hpp"

#include "../io.hpp"

#include "./detail.hpp"
#include "./merge.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

namespace {

inline std::vector<TileIndex> tileIndices(const TileSet::const_ptrlist &sets
                                          , const LodRange &lodRange)
{
    std::vector<TileIndex> indices;
    for (const auto ts : sets) {
        indices.push_back
            (ts->tileIndex(lodRange).simplify(TileIndex::Flag::mesh));
    }
    return indices;
}

inline TileIndices tileIndices(const std::vector<TileIndex> &tis
                               , std::size_t size
                               = std::numeric_limits<std::size_t>::max())
{
    size = std::min(size, tis.size());

    TileIndices indices;
    for (const auto &ti : tis) {
        indices.push_back(&ti);
    }

    return indices;
}

LodRange range(const TileSet::const_ptrlist &sets)
{
    // start with invalid range
    LodRange r(LodRange::emptyRange());

    if (sets.empty()) { return r; }

    for (const auto &set : sets) {
        r = unite(r, set->lodRange());
    }
    return r;
}

const char *TILEINDEX_DUMP_ROOT("TILEINDEX_DUMP_ROOT");

const char* getDumpDir()
{
    return std::getenv(TILEINDEX_DUMP_ROOT);
}

void dumpTileIndex(const char *root, const fs::path &name
                   , const TileIndex &index)
{
    if (!root) { return; }
    LOG(info1) << "Dumping tileindex " << name << ".";
    dumpAsImages(root / name, index);
}

inline void dump(const char *root, const boost::filesystem::path &dir
                 , const std::vector<TileIndex> &tileIndices)
{
    if (!root) { return; }

    int i(0);
    for (const auto &ti : tileIndices) {
        dumpTileIndex(root, dir / str(boost::format("%03d") % i), ti);
        ++i;
    }
}

struct Merger {
    Merger(TileSet::Detail &self, const TileIndex &generate
           , const TileSet::const_ptrlist &src)
        : self(self), world(generate), generate(generate)
        , src(src), progress(generate.count())
    {
        // make world complete
        world.complete();
    }

    /** Merge subtree starting at index.
     *  Calls itself recursively.
     */
    void mergeTile(const TileId &tileId = TileId()
                   , const merge::Input::list &parentSource
                   = merge::Input::list()
                   , int quadrant = -1
                   , bool parentGenerated = false);

    /** Generates new tile as a merge of tiles from other tilesets.
     */
    void generateTile(const TileId &tileId
                      , const merge::Input::list &parentSource
                      , merge::Input::list &source, int quadrant);


    TileSet::Detail &self;
    TileIndex world;
    const TileIndex &generate;
    const TileSet::const_ptrlist &src;

    utility::Progress progress;
};

void Merger::mergeTile(const TileId &tileId
                       , const merge::Input::list &parentSource
                       , int quadrant, bool parentGenerated)
{
    if (!world.exists(tileId)) {
        // no data below
        return;
    }

    merge::Input::list source;

    auto g(generate.exists(tileId));
    if (g) {
        LOG(info2) << "(glue) Processing tile " << tileId << ".";

        bool thisGenerated(false);

        if (!parentGenerated) {
            // if (auto t = self.getTile(self.parent(tileId), std::nothrow)) {
            //     // no parent was generated and we have sucessfully loaded parent
            //     // tile from existing content as a fallback tile!

            //     // TODO: since this is merge result its information can be
            //     // outdated
            //     MergeInput::list loadedParentTiles;
            //     loadedParentTiles.push_back
            //         (MergeInput(t.get(), nullptr, tileId));

            //     quadrant = child(tileId);
            //     tile = generateTile(tileId, loadedParentTiles
            //                         , incidentTiles, quadrant);
            //     thisGenerated = true;
            // }
        }

        if (!thisGenerated) {
            // regular generation
            generateTile(tileId, parentSource, source, quadrant);
        }

        (++progress).report(utility::Progress::ratio_t(5, 1000), "(glue) ");
    }

    // can we go down?
    if (tileId.lod >= generate.maxLod()) {
        // no way down
        return;
    }

    // OK, process children
    quadrant = 0;
    for (const auto &child : children(tileId)) {
        mergeTile(child, source, quadrant++, g);
    }
}

void Merger::generateTile(const TileId &tileId
                          , const merge::Input::list &parentSource
                          , merge::Input::list &source, int quadrant)
{
    (void) tileId;
    (void) parentSource;
    (void) source;
    (void) quadrant;

    LOG(info4) << "Generate tile: " << tileId;

    // create input
    merge::Input::list input;
    for (const auto &ts : src) {
        merge::Input t(self.other(*ts), tileId);
        if (t) { input.push_back(t); }
    }

    // TODO: physical tile merge

    // TODO: analyze tile and store if it is proper glue tile
}

} // namespace

void TileSet::createGlue(const const_ptrlist &sets)
{
    LOG(info3) << "(glue) Calculating generate set.";

    const auto *dumpRoot(getDumpDir());

    // lod range of the world
    auto lodRange(range(sets));

    // clone simplified indices
    auto indices(tileIndices(sets, lodRange));
    dump(dumpRoot, "indices", indices);

    auto tsUpdate(indices.back());
    if (tsUpdate.empty()) {
        LOG(warn3) << "(glue) Nothing to generate. Bailing out.";
        return;
    }

    dumpTileIndex(dumpRoot, "tsUpdate", tsUpdate);
    tsUpdate.growDown();
    dumpTileIndex(dumpRoot, "tsUpdate-gd", tsUpdate);

    auto tsPost(unite(tileIndices(indices)));
    dumpTileIndex(dumpRoot, "tsPost", tsPost);
    tsPost.growUp();
    dumpTileIndex(dumpRoot, "tsPost-gu", tsPost);

    auto tsPre(unite(tileIndices(indices, indices.size() - 1)));
    dumpTileIndex(dumpRoot, "tsPre", tsPre);
    tsPre.growUp();
    dumpTileIndex(dumpRoot, "tsPre-gu", tsPre);
    tsPre.invert();
    dumpTileIndex(dumpRoot, "tsPre-gu-inv", tsPre);

    auto generate(tsPost.intersect(unite(tsUpdate, tsPre)));
    dumpTileIndex(dumpRoot, "generate", generate);

    LOG(info1) << "generate: " << generate.count();

    if (generate.empty()) {
        LOG(warn3) << "(glue) Nothing to generate. Bailing out.";
        return;
    }

    Merger(detail(), generate, sets).mergeTile();
}

} } // namespace vadstena::vts
