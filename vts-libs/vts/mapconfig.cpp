/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <set>

#include <boost/filesystem/path.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "../registry/json.hpp"
#include "../storage/error.hpp"

#include "./tileop.hpp"
#include "./mapconfig.hpp"
#include "./mapconfig-json.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

namespace {
    const int VERSION = 1;
} // namespace

const char* MapConfig::contentType("application/json; charset=utf-8");
const char* MeshTilesConfig::contentType("application/json; charset=utf-8");
const char* DebugConfig::contentType("application/json; charset=utf-8");

Json::Value asJson(const Glue::Id &id)
{
    Json::Value value(Json::arrayValue);
    for (const auto &str : id) { value.append(str); }
    return value;
}

void addRanges(const SurfaceCommonConfig &surface, Json::Value &s)
{
    auto &lodRange(s["lodRange"] = Json::arrayValue);
    lodRange.append(surface.lodRange.min);
    lodRange.append(surface.lodRange.max);

    auto &tileRange(s["tileRange"] = Json::arrayValue);
    auto &tileRangeMin(tileRange.append(Json::arrayValue));
    tileRangeMin.append(surface.tileRange.ll(0));
    tileRangeMin.append(surface.tileRange.ll(1));
    auto &tileRangeMax(tileRange.append(Json::arrayValue));
    tileRangeMax.append(surface.tileRange.ur(0));
    tileRangeMax.append(surface.tileRange.ur(1));
}

void asJson(const SurfaceCommonConfig &surface, Json::Value &s
            , registry::BoundLayer::dict &boundLayers)
{
    addRanges(surface, s);

    // paths
    if (surface.urls3d) {
        s["metaUrl"] = surface.urls3d->meta;
        s["meshUrl"] = surface.urls3d->mesh;
        s["textureUrl"] = surface.urls3d->texture;
        s["navUrl"] = surface.urls3d->nav;
    } else {
        s["metaUrl"]
            = (surface.root / fileTemplate(storage::TileFile::meta
                                           , surface.revision)).string();
        s["meshUrl"]
            = (surface.root / fileTemplate(storage::TileFile::mesh
                                           , surface.revision)).string();
        s["textureUrl"]
            = (surface.root / fileTemplate(storage::TileFile::atlas
                                           , surface.revision)).string();
        s["navUrl"]
            = (surface.root / fileTemplate(storage::TileFile::navtile
                                           , surface.revision)).string();
    }

    if (surface.has2dInterface) {
        auto &i2d(s["2d"] = Json::objectValue);
        if (surface.urls2d) {
            s["metaUrl"] = surface.urls2d->meta;
            s["maskUrl"] = surface.urls2d->mask;
            s["orthoUrl"] = surface.urls2d->ortho;
            s["creditsUrl"] = surface.urls2d->credits;
        } else {
            i2d["metaUrl"]
                = (surface.root / fileTemplate(storage::TileFile::meta2d
                                               , surface.revision)).string();
            i2d["maskUrl"]
                = (surface.root / fileTemplate(storage::TileFile::mask
                                               , surface.revision)).string();
            i2d["orthoUrl"]
                = (surface.root / fileTemplate(storage::TileFile::ortho
                                               , surface.revision)).string();
            i2d["creditsUrl"]
                = (surface.root / fileTemplate(storage::TileFile::credits
                                               , surface.revision)).string();
        }
    }

    if (surface.textureLayer) {
        s["textureLayer"] = *surface.textureLayer;
        boundLayers.add
            (registry::system.boundLayers(*surface.textureLayer));
    }
}

Json::Value asJson(const SurfaceConfig &surface
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::objectValue);
    s["id"] = surface.id;
    asJson(surface, s, boundLayers);
    return s;
}

Json::Value asJson(const GlueConfig &glue
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::objectValue);
    s["id"] = asJson(glue.id);
    asJson(glue, s, boundLayers);
    return s;
}

Json::Value asJson(const VirtualSurfaceConfig &vs)
{
    Json::Value s(Json::objectValue);
    s["id"] = asJson(vs.id);
    addRanges(vs, s);

    // paths
    if (vs.urls3d) {
        s["metaUrl"] = vs.urls3d->meta;
    } else {
        s["metaUrl"]
            = (vs.root / fileTemplate(storage::TileFile::meta
                                      , vs.revision)).string();
    }

    if (!vs.mapping.empty()) {
        s["mapping"] = vs.mapping;
    } else {
        s["mapping"] = (vs.root / VirtualSurface::TilesetMappingPath).string();
    }

    return s;
}

Json::Value asJson(const SurfaceConfig::list &surfaces
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::arrayValue);
    for (const auto &surface : surfaces) {
        s.append(asJson(surface, boundLayers));
    }
    return s;
}

Json::Value asJson(const GlueConfig::list &glues
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::arrayValue);
    for (const auto &glue : glues) {
        s.append(asJson(glue, boundLayers));
    }
    return s;
}

Json::Value asJson(const VirtualSurfaceConfig::list &virtualSurfaces)
{
    Json::Value s(Json::arrayValue);
    for (const auto &virtualSurface : virtualSurfaces) {
        s.append(asJson(virtualSurface));
    }
    return s;
}

// parsing

void fromJson(Glue::Id &id, const Json::Value &value)
{
    for (const auto &element : value) {
        Json::check(element, Json::stringValue);
        id.push_back(element.asString());
    }
}

void fromJson(SurfaceCommonConfig &surface, const Json::Value &value)
{
    Json::get(surface.lodRange.min, value, "lodRange", 0);
    Json::get(surface.lodRange.max, value, "lodRange", 1);

    surface.tileRange = registry::tileRangeFromJson(value["tileRange"]);

    surface.urls3d = boost::in_place();
    Json::get(surface.urls3d->meta, value, "metaUrl");
    Json::get(surface.urls3d->mesh, value, "meshUrl");
    Json::get(surface.urls3d->texture, value, "textureUrl");
    Json::get(surface.urls3d->nav, value, "navUrl");

    if (value.isMember("2d")) {
        surface.has2dInterface = true;
        const auto &i2d(value["2d"]);
        surface.urls2d = boost::in_place();
        Json::get(surface.urls2d->meta, i2d, "metaUrl");
        Json::get(surface.urls2d->mask, i2d, "maskUrl");
        Json::get(surface.urls2d->ortho, i2d, "orthoUrl");
        Json::get(surface.urls2d->credits, i2d, "creditsUrl");
    }

    if (value.isMember("textureLayer")) {
        surface.textureLayer = boost::in_place();
        Json::get(*surface.textureLayer, value, "textureLayer");
    }
}

void fromJson(SurfaceConfig &surface, const Json::Value &value)
{
    Json::get(surface.id, value, "id");
    fromJson(static_cast<SurfaceCommonConfig&>(surface), value);
}

void fromJson(GlueConfig &glue, const Json::Value &value)
{
    fromJson(glue.id, value["id"]);
    fromJson(static_cast<SurfaceCommonConfig&>(glue), value);
}

void fromJson(VirtualSurfaceConfig &vs, const Json::Value &value)
{
    fromJson(vs.id, value["id"]);

    Json::get(vs.lodRange.min, value, "lodRange", 0);
    Json::get(vs.lodRange.max, value, "lodRange", 1);

    vs.tileRange = registry::tileRangeFromJson(value["tileRange"]);

    vs.urls3d = boost::in_place();
    Json::get(vs.urls3d->meta, value, "metaUrl");
    Json::get(vs.mapping, value, "mapping");
}

void fromJson(SurfaceConfig::list &surfaces
              , const Json::Value &value)
{
    for (const auto &surface : value) {
        surfaces.emplace_back();
        fromJson(surfaces.back(), surface);
    }
}

void fromJson(GlueConfig::list &glues
              , const Json::Value &value)
{
    for (const auto &glue : value) {
        glues.emplace_back();
        fromJson(glues.back(), glue);
    }
}

void fromJson(VirtualSurfaceConfig::list &vss
              , const Json::Value &value)
{
    for (const auto &vs : value) {
        vss.emplace_back();
        fromJson(vss.back(), vs);
    }
}

void mergeRest(MapConfig &out, const MapConfig &in, bool surface)
{
    out.srs.update(in.srs);
    out.credits.update(in.credits);
    out.boundLayers.update(in.boundLayers);
    out.freeLayers.update(in.freeLayers);

    // merge views
    if (surface) {
        out.view.merge(in.view);

        // TODO: find out first valid
        out.position = in.position;
    }

    // all inputs must be texture-atlas-ready
    out.textureAtlasReady &= in.textureAtlasReady;
}

void MapConfig::mergeTileSet(const MapConfig &tilesetMapConfig
                             , const boost::filesystem::path &root)
{
    if (tilesetMapConfig.surfaces.size() != 1) {
        LOGTHROW(err1, storage::NoSuchTileSet)
            << "Cannot merge tileset mapConfig: "
            "there must be just one surface in the input mapConfig.";
    }

    SurfaceConfig s(tilesetMapConfig.surfaces.front());
    // set root if not set so far
    if (s.root.empty()) { s.root = root; }
    surfaces.push_back(s);

    mergeRest(*this, tilesetMapConfig, true);
}

void MapConfig::addMeshTilesConfig(const MeshTilesConfig &meshTilesConfig
                                   , const boost::filesystem::path &root)
{
    meshTiles.push_back(meshTilesConfig);
    auto &s(meshTiles.back().surface);
    if (s.root.empty()) { s.root = root; }
}

void MapConfig::mergeGlue(const MapConfig &tilesetMapConfig
                          , const Glue &glue
                          , const boost::filesystem::path &root)
{
    if (tilesetMapConfig.surfaces.size() != 1) {
        LOGTHROW(err1, storage::NoSuchTileSet)
            << "Cannot merge tileset mapConfig as a glue: "
            "there must be just one surface in the input mapConfig.";
    }

    GlueConfig g(tilesetMapConfig.surfaces.front());
    g.id = glue.id;
    g.root = root / glue.path;
    glues.push_back(g);

    mergeRest(*this, tilesetMapConfig, false);
}

void MapConfig::mergeVirtualSurface(const MapConfig &tilesetMapConfig
                                    , const VirtualSurface &virtualSurface
                                    , const boost::filesystem::path &root)
{
    if (tilesetMapConfig.surfaces.size() != 1) {
        LOGTHROW(err1, storage::NoSuchTileSet)
            << "Cannot merge tileset mapConfig as a virtual surface: "
            "there must be just one surface in the input mapConfig.";
    }

    VirtualSurfaceConfig vs(tilesetMapConfig.surfaces.front());
    vs.id = virtualSurface.id;
    vs.root = root / virtualSurface.path;
    virtualSurfaces.push_back(vs);

    mergeRest(*this, tilesetMapConfig, false);
}

void MapConfig::merge(const MapConfig &other)
{
    if (referenceFrame.id.empty()) {
        // assign reference frame
        if (other.referenceFrame.id.empty()) {
            LOGTHROW(err1, storage::InconsistentInput)
                << "Missing referenceFrame while merging "
                "map configuration.";
        }
        referenceFrame = other.referenceFrame;
    } else if (referenceFrame.id != other.referenceFrame.id) {
        // mismatch
        LOGTHROW(err1, storage::InconsistentInput)
            << "Cannot merge map configuration for reference frame <"
            << referenceFrame.id
            << "> with map configuration for reference frame <"
            << other.referenceFrame.id << ">.";
    }

    surfaces.insert(surfaces.end(), other.surfaces.begin()
                    , other.surfaces.end());
    glues.insert(glues.end(), other.glues.begin(), other.glues.end());
    virtualSurfaces.insert(virtualSurfaces.end(), other.virtualSurfaces.begin()
                           , other.virtualSurfaces.end());
    mergeRest(*this, other, true);
}

void saveMapConfig(const MapConfig &mapConfig, std::ostream &os)
{
    Json::Value content;
    content["version"] = VERSION;

    content["srses"] = registry::asJson(mapConfig.srs, true);
    content["referenceFrame"] = registry::asJson(mapConfig.referenceFrame);

    auto boundLayers(mapConfig.boundLayers);

    // get credits, append all from bound layers
    auto credits(mapConfig.credits);

    content["surfaces"] = asJson(mapConfig.surfaces, boundLayers);
    content["glue"] = asJson(mapConfig.glues, boundLayers);
    content["virtualSurfaces"] = asJson(mapConfig.virtualSurfaces);

    content["position"] = registry::asJson(mapConfig.position);

    content["freeLayers"] = registry::asJson(mapConfig.freeLayers);
    content["rois"] = registry::asJson(mapConfig.rois);
    content["view"] = registry::asJson(mapConfig.view, boundLayers);
    content["namedViews"]
        = registry::asJson(mapConfig.namedViews, boundLayers);;

    // fill in free tilesets as free layers
    for (const auto &config : mapConfig.meshTiles) {
        // no inline credits
        content["freeLayers"][config.surface.id]
            = asJson(freeLayer(config, false), false);
        // and fetch credits
        credits.update(config.credits);
    }

    // dunno what to put here...
    content["params"] = Json::objectValue;

    for (const auto &bl : boundLayers) {
        credits.update(registry::creditsAsDict(bl.credits));
    }

    // grab credits
    content["credits"] = registry::asJson(credits);
    // get bound layers, no inline credits
    content["boundLayers"] = registry::asJson(boundLayers, false);

    content["textureAtlasReady"] = mapConfig.textureAtlasReady;

    // add browser core options if present
    if (!mapConfig.browserOptions.empty()) {
        try {
            content["browserOptions"]
                = boost::any_cast<Json::Value>(mapConfig.browserOptions);
        } catch (boost::bad_any_cast) {
            // ignore
        }
    }

    os.precision(15);
    Json::write(os, content);
}

namespace detail {

void parse1(MapConfig &mapConfig, const Json::Value &config)
{
    fromJson(mapConfig.srs, config["srses"]);
    fromJson(mapConfig.referenceFrame, config["referenceFrame"]);
    fromJson(mapConfig.credits, config["credits"]);
    fromJson(mapConfig.boundLayers, config["boundLayers"]);
    fromJson(mapConfig.freeLayers, config["freeLayers"]);

    fromJson(mapConfig.surfaces, config["surfaces"]);
    fromJson(mapConfig.glues, config["glue"]);

    if (config.isMember("virtualSurfaces")) {
        // virtual surfaces are optional
        fromJson(mapConfig.virtualSurfaces, config["virtualSurfaces"]);
    }

    fromJson(mapConfig.view, config["view"]);
    fromJson(mapConfig.namedViews, config["namedViews"]);

    mapConfig.position = registry::positionFromJson(config["position"]);

    if (config.isMember("browserOptions")) {
        mapConfig.browserOptions = config["browserOptions"];
    }
}

} // namespace detail

Json::Value browserOptions(const MapConfig &mapConfig)
{
    if (mapConfig.browserOptions.empty()) {
        return Json::Value(Json::nullValue);
    }

    try {
        return boost::any_cast<Json::Value>(mapConfig.browserOptions);
    } catch (boost::bad_any_cast) {
        // ignore
        return Json::Value(Json::nullValue);
    }
}

void loadMapConfig(MapConfig &mapConfig, std::istream &in
                   , const boost::filesystem::path &path)
{
    // load json
    auto config(Json::read<vtslibs::storage::FormatError>
                (in, path, "map config"));

    try {
        int version(0);
        Json::get(version, config, "version");

        switch (version) {
        case 1:
            detail::parse1(mapConfig, config);
            return;
        }

        LOGTHROW(err1, storage::FormatError)
            << "Invalid map config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, storage::FormatError)
            << "Invalid map config format (" << e.what()
            << ") in " << path << ".";
    }
    throw;
}

registry::FreeLayer freeLayer(const MeshTilesConfig &config
                              , bool inlineCredits
                              , const boost::filesystem::path &root)
{
    registry::FreeLayer fl;

    fl.id = config.surface.id;
    for (const auto &credit : config.credits) {
        if (inlineCredits) {
            fl.credits.set(credit.id, credit);
        } else {
            fl.credits.set(credit.id, boost::none);
        }
    }

    auto &def(fl.createDefinition<registry::FreeLayer::MeshTiles>());

    const auto &surface(config.surface);
    def.lodRange = surface.lodRange;
    def.tileRange = surface.tileRange;

    auto useRoot(surface.root.empty() ? root : surface.root);

    def.metaUrl
        = (useRoot / fileTemplate(storage::TileFile::meta
                                  , surface.revision)).string();
    def.meshUrl
        = (useRoot / fileTemplate(storage::TileFile::mesh
                                  , surface.revision)).string();
    def.textureUrl
        = (useRoot / fileTemplate(storage::TileFile::atlas
                                  , surface.revision)).string();
    // done
    return fl;
}

DebugConfig debugConfig(const MeshTilesConfig &config
                        , const std::string &referenceFrameId
                        , const boost::filesystem::path &root)
{
    DebugConfig dc;
    const auto &surface(config.surface);
    dc.referenceFrame = referenceFrameId;
    dc.lodRange = surface.lodRange;
    dc.tileRange = surface.tileRange;

    const auto useRoot(surface.root.empty() ? root : surface.root);

    dc.meta
        = (useRoot / fileTemplate(storage::TileFile::meta, FileFlavor::debug
                                  , surface.revision)).string();
    dc.mask
        = (useRoot / fileTemplate(storage::TileFile::mask, FileFlavor::debug
                                  , surface.revision)).string();
    return dc;
}

namespace {

typedef std::string ResourceId;

typedef std::vector<ResourceId> Resources;

typedef std::map<std::string, Resources> Dirs;

inline void update(Dirs &dirs, const std::string &path
                   , const Resources &resources)
{
    auto &dst(dirs[path]);
    dst.insert(dst.begin(), resources.begin(), resources.end());
}

inline void update(Dirs &dirs, const std::string &path
                   , const ResourceId &resourceId)
{
    dirs[path].push_back(resourceId);
}

inline void addBase(Dirs &dirs, const fs::path &path
                    , const ResourceId &resourceId)
{
    if (path.has_filename()) {
        auto tmp(path.parent_path());
        if (tmp.empty()) {
            update(dirs, ".", resourceId);
        } else {
            update(dirs, tmp.string(), resourceId);
        }
    } else {
        update(dirs, path.string(), resourceId);
    }
}

inline void addBase(Dirs &dirs, const std::string &path
                    , const ResourceId &resourceId)
{
    addBase(dirs, fs::path(path), resourceId);
}

inline void addBase(Dirs &dirs, const boost::optional<std::string> &path
                    , const ResourceId &resourceId)
{
    if (path) { addBase(dirs, fs::path(*path), resourceId); }
}

inline void extractDirs(Dirs &dirs, const SurfaceConfig &surface
                        , std::set<std::string> &boundLayers)
{
    if (surface.root.empty()) {
        update(dirs, ".", surface.id);
    } else {
        update(dirs, surface.root.string(), surface.id);
    }
    if (surface.textureLayer) {
        boundLayers.insert(*surface.textureLayer);
    }
}

inline void extractDirs(Dirs &dirs, const GlueConfig &surface
                        , std::set<std::string> &boundLayers)
{
    if (surface.root.empty()) {
        update(dirs, ".", surface.id);
    } else {
        update(dirs, surface.root.string(), surface.id);
    }
    if (surface.textureLayer) {
        boundLayers.insert(*surface.textureLayer);
    }
}

inline void extractDirs(Dirs &dirs, const VirtualSurfaceConfig &surface
                        , std::set<std::string> &boundLayers)
{
    if (surface.root.empty()) {
        update(dirs, ".", surface.id);
    } else {
        update(dirs, surface.root.string(), surface.id);
    }
    if (surface.textureLayer) {
        boundLayers.insert(*surface.textureLayer);
    }
}

class ExtractVisitor : public boost::static_visitor<> {
public:
    ExtractVisitor(Dirs &dirs, const ResourceId &id)
        : dirs_(dirs), id_(id) {}

    void operator()(const std::string &def) {
        addBase(dirs_, def, id_);
    }

    void operator()(const registry::FreeLayer::Geodata &def) {
        addBase(dirs_, def.geodata, id_);
        addBase(dirs_, def.style, id_);
    }

    void operator()(const registry::FreeLayer::GeodataTiles &def) {
        addBase(dirs_, def.metaUrl, id_);
        addBase(dirs_, def.geodataUrl, id_);
        addBase(dirs_, def.style, id_);
    }

    void operator()(const registry::FreeLayer::MeshTiles &def) {
        addBase(dirs_, def.metaUrl, id_);
        addBase(dirs_, def.meshUrl, id_);
        addBase(dirs_, def.textureUrl, id_);
    }

private:
    Dirs &dirs_;
    const ResourceId &id_;
};

inline void extractDirs(Dirs &dirs, const registry::BoundLayer &bl)
{
    addBase(dirs, bl.url, bl.id);
    addBase(dirs, bl.maskUrl, bl.id);
    addBase(dirs, bl.metaUrl, bl.id);
    addBase(dirs, bl.creditsUrl, bl.id);
}

Dirs extractDirs(const MapConfig &mapConfig)
{
    Dirs dirs;
    std::set<std::string> boundLayers;

    // add this directory (whatever it is)
    dirs["."];

    for (const auto &surface : mapConfig.surfaces) {
        extractDirs(dirs, surface, boundLayers);
    }

    for (const auto &glue : mapConfig.glues) {
        extractDirs(dirs, glue, boundLayers);
    }

    for (const auto &virtualSurface : mapConfig.virtualSurfaces) {
        extractDirs(dirs, virtualSurface, boundLayers);
    }

    for (const auto &meshTiles : mapConfig.meshTiles) {
        extractDirs(dirs, meshTiles.surface, boundLayers);
    }

    for (const auto &pair : mapConfig.freeLayers) {
        ExtractVisitor ev(dirs, pair.first);
        boost::apply_visitor(ev, pair.second.definition);
    }

    for (const auto &bl : mapConfig.boundLayers) {
        extractDirs(dirs, bl);
    }

    // extra bound layers
    for (const auto &blId : boundLayers) {
        if (const auto *bl
            = registry::system.boundLayers(blId, std::nothrow))
        {
            extractDirs(dirs, *bl);
        }
    }

    return dirs;
}

} // namespace

void saveDirs(const MapConfig &mapConfig, std::ostream &os)
{
    Json::Value content(Json::objectValue);

    for (const auto &item : extractDirs(mapConfig)) {
        auto &resources(content[item.first] = Json::arrayValue);
        for (const auto &rid : item.second) {
            resources.append(rid);
        }
    }

    Json::write(os, content);
}


void asJson(const DebugConfig &debug, Json::Value &content)
{
    content["referenceFrame"] = debug.referenceFrame;
    auto &lodRange(content["lodRange"] = Json::arrayValue);
    lodRange.append(debug.lodRange.min);
    lodRange.append(debug.lodRange.max);

    auto &tileRange(content["tileRange"] = Json::arrayValue);
    auto &tileRangeMin(tileRange.append(Json::arrayValue));
    tileRangeMin.append(debug.tileRange.ll(0));
    tileRangeMin.append(debug.tileRange.ll(1));
    auto &tileRangeMax(tileRange.append(Json::arrayValue));
    tileRangeMax.append(debug.tileRange.ur(0));
    tileRangeMax.append(debug.tileRange.ur(1));

    content["metaUrl"] = debug.meta;
    content["maskUrl"] = debug.mask;
}

void saveDebug(std::ostream &os, const DebugConfig &debug)
{
    Json::Value content(Json::objectValue);
    asJson(debug, content);
    os.precision(15);
    Json::write(os, content);
}

} } // namespace vtslibs::vts
