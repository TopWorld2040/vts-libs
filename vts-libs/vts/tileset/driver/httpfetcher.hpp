#ifndef vadstena_libs_vts_tileset_driver_httpfetcher_hpp_included_
#define vadstena_libs_vts_tileset_driver_httpfetcher_hpp_included_

#include <memory>

#include "../driver.hpp"

namespace vadstena { namespace vts { namespace driver {

class HttpFetcher {
public:
    struct Options {};

    HttpFetcher(const std::string &rootUrl, const Options &options);

    IStream::pointer input(File type) const;

    IStream::pointer input(const TileId &tileId, TileFile type
                           , unsigned int revision) const;

private:
    const std::string &rootUrl_;
    Options options_;
    std::shared_ptr<void> handle_;
};

} } } // namespace vadstena::vts::driver

#endif // vadstena_libs_vts_tileset_driver_httpfetcher_hpp_included_