/**
 * \file vts/storage.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Store.
 */

#ifndef vadstena_libs_vts_storage_hpp_included_
#define vadstena_libs_vts_storage_hpp_included_

#include <memory>
#include <map>

#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/runnable.hpp"

#include "./properties.hpp"

namespace vadstena { namespace vts {

/** Tile set descriptor.
 */
struct TileSetDescriptor {
    Locator locator;

    typedef std::map<std::string, TileSetDescriptor> map;

    TileSetDescriptor() {}
};

/** Storage properties.
 */
struct StorageProperties {
    /** Set of input tile set descriptors.
     */
    TileSetDescriptor::map inputSets;

    /** Output tile sets.
     */
    TileSetDescriptor outputSet;

    StorageProperties() {}
};

/** Storage create properties.
 */
struct StorageCreateProperties {
    /** Output tile set create properties.
     */
    CreateProperties createProperties;

    std::string outputTileSetType;

    StorageCreateProperties() {}

    StorageCreateProperties(const CreateProperties &createProperties)
        : createProperties(createProperties)
    {}

    StorageCreateProperties(const CreateProperties &createProperties
                            , const std::string &outputTileSetType)
        : createProperties(createProperties)
        , outputTileSetType(outputTileSetType)
    {}
};

class Storage : boost::noncopyable
{
public:
    typedef std::shared_ptr<Storage> pointer;

    StorageProperties getProperties() const;

    void addTileSet(const Locator &locator
                    , utility::Runnable *runnable = nullptr);

    void addTileSets(const std::vector<Locator> &locators
                     , utility::Runnable *runnable = nullptr);

    void rebuildOutput();

    void removeTileSet(const std::string &id
                       , utility::Runnable *runnable = nullptr);

    void removeTileSets(const std::vector<std::string> &ids
                        , utility::Runnable *runnable = nullptr);

    static std::map<std::string, std::string> listSupportedDrivers();

    static const std::string getDefaultOutputType();

    /** Needed to instantiate.
     */
    class Factory;
    friend class Factory;

private:
    Storage(const boost::filesystem::path &root, bool readOnly);

    struct Detail;
    std::unique_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};


// inline stuff

inline void Storage::addTileSet(const Locator &locator
                                , utility::Runnable *runnable)
{
    return addTileSets({locator}, runnable);
}

inline void Storage::removeTileSet(const std::string &id
                                   , utility::Runnable *runnable)
{
    return removeTileSets({id}, runnable);
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_storage_hpp_included_