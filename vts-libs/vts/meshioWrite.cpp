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

#include "meshioCommon.hpp"

namespace vtslibs { namespace vts { namespace detail {

namespace {

const char *NO_MESH_COMPRESSION(std::getenv("NO_MESH_COMPRESSION"));

/** Calculate Forsyth's cache optimized ordering of faces, vertices and
 *  texcoords. On return, forder, vorder, torder contain a new index for each
 *  face, vertex and texcoord, respectively.
 */
void getMeshOrdering(const SubMesh &submesh,
                     std::vector<int> &forder,
                     std::vector<int> &vorder,
                     std::vector<int> &torder)
{
    int nfaces = submesh.faces.size();
    int nvertices = submesh.vertices.size();
    int ntexcoords = submesh.tc.size();

#if 1
    std::vector<geometry::ForsythVertexIndexType> indices;
    indices.reserve(3*nfaces);
    for (const auto &face : submesh.faces) {
        indices.push_back(face(0));
        indices.push_back(face(1));
        indices.push_back(face(2));
    }

    // get face ordering with Forsyth's algorithm
    forder.resize(nfaces);
    geometry::forsythReorder(forder.data(), indices.data(), nfaces, nvertices);
    // TODO: forsythReorder currently does not take texcoords into account!
#else
    // for testing lack of ordering
    forder.resize(nfaces);
    for (int i = 0; i < nfaces; i++) { forder[i] = i; }
#endif

    vorder.assign(nvertices, -1);
    torder.assign(ntexcoords, -1);

    // face ordering induces vertex and texcoord ordering
    int vnext = 0, tnext = 0;
    for (int i = 0; i < nfaces; i++) {

        const auto &face(submesh.faces[forder[i]]);
        for (int j = 0; j < 3; j++) {
            int &vindex = vorder[face(j)];
            if (vindex < 0) { vindex = vnext++; }
        }

        if (submesh.facesTc.size()) {
            const auto &facetc(submesh.facesTc[forder[i]]);
            for (int j = 0; j < 3; j++) {
                int &tindex = torder[facetc(j)];
                if (tindex < 0) { tindex = tnext++; }
            }
        }
    }
    assert(vnext == nvertices);
    assert(tnext == ntexcoords);
}

/** Helper class to write 16-bit signed/unsigned deltas variably as 1-2 bytes
 *  and collect statistics.
 */
class DeltaWriter
{
public:
    DeltaWriter(std::ostream &out)
        : out_(out), nbytes_(), nsmall_(), nbig_() {}

    void writeWord(unsigned word)
    {
        // TODO: throw exception instead
        assert(word < (1 << 15));
        if (word < 0x80) {
            bin::write(out_, std::uint8_t(word));
            nsmall_++;
            nbytes_++;
        }
        else {
            bin::write(out_, std::uint8_t(0x80 | (word & 0x7f)));
            bin::write(out_, std::uint8_t(word >> 7));
            nbig_++;
            nbytes_ += 2;
        }
    }

    void writeDelta(int value, int &last)
    {
        signed delta = value - last;
        unsigned zigzag = (delta << 1) ^ (delta >> 31);
        writeWord(zigzag);
        last = value;
    }

    int nbytes() const { return nbytes_; }
    int nsmall() const { return nsmall_; }
    int nbig() const { return nbig_; }

private:
    std::ostream &out_;
    int nbytes_, nsmall_, nbig_;
};

void invertIndex(const std::vector<int> &in, std::vector<int> &out)
{
    out.resize(in.size());
    for (unsigned i = 0; i < in.size(); i++) {
        out[in[i]] = i;
    }
}

/** Returns size of submesh texture at given index. If atlas is null or there is
 *  no such texture, default size is returned.
 */
math::Size2 textureSize(const Atlas *atlas, std::size_t submesh
                        , const math::Size2 &dflt)
{
    // fallback to default
    if (!atlas || (submesh >= atlas->size())) { return dflt; }

    return atlas->imageSize(submesh);
}

void saveMeshVersion3(std::ostream &out, const Mesh &mesh
                      , const Atlas *atlas)
{
    // write header
    bin::write(out, MAGIC);
    bin::write(out, std::uint16_t(3));

    // no mean undulation
    bin::write(out, double(0.0));

    bin::write(out, std::uint16_t(mesh.submeshes.size()));

    // write submeshes
    int smIndex(-1);
    for (const SubMesh &submesh : mesh)
    {
        smIndex++;

        // remove degenerate faces
        SubMesh sm(submesh.cleanUp());

        // get a good ordering of faces, vertices and texcoords
        std::vector<int> forder, vorder, torder;
        getMeshOrdering(sm, forder, vorder, torder);

        auto bbox(extents(sm));
        math::Point3d bbsize(bbox.ur - bbox.ll);
        math::Point3d center(0.5*(bbox.ll + bbox.ur));
        double scale(1.0 / std::max(bbsize(0), std::max(bbsize(1), bbsize(2))));

        // build and write flags
        std::uint8_t flags(0);
        if (!sm.tc.empty()) {
            flags |= SubMeshFlag::internalTexture;
        }
        if (!sm.etc.empty()) {
            flags |= SubMeshFlag::externalTexture;
        }
        if (sm.textureMode == SubMesh::TextureMode::external) {
            flags |= SubMeshFlag::textureMode;
        }
        bin::write(out, flags);

        // surface reference (defaults to 1)
        bin::write(out, std::uint8_t(sm.surfaceReference));

        // write (external) texture layer information
        if (sm.textureLayer) {
            bin::write(out, std::uint16_t(*sm.textureLayer));
        } else {
            // save zero
            bin::write(out, std::uint16_t(0));
        }

        // write extents
        bin::write(out, bbox.ll(0));
        bin::write(out, bbox.ll(1));
        bin::write(out, bbox.ll(2));
        bin::write(out, bbox.ur(0));
        bin::write(out, bbox.ur(1));
        bin::write(out, bbox.ur(2));

        DeltaWriter dw(out);

        // write delta coded vertices
        int b1 = dw.nbytes();
        {
            // TODO: check: nv must be < 2^16
            int nv(sm.vertices.size());
            bin::write(out, std::uint16_t(nv));
            bin::write(out, std::uint16_t(GeomQuant));

            std::vector<int> ivorder;
            invertIndex(vorder, ivorder);

            int last[3] = {0, 0, 0};
            for (int i = 0; i < nv; i++) {
                const auto &v(sm.vertices[ivorder[i]]);
                for (int j = 0; j < 3; j++)
                {
                    double ncoord = (v(j) - center(j)) * scale;
                    int qcoord = round(ncoord * GeomQuant);
                    dw.writeDelta(qcoord, last[j]);
                }
            }

            // write delta coded external coordinates
            if (flags & SubMeshFlag::externalTexture)
            {
                int quant = (int(vtslibs::registry::BoundLayer::tileWidth)
                             << TexCoordSubPixelBits);
                bin::write(out, std::uint16_t(quant));

                int last[2] = {0, 0};
                for (int i = 0; i < nv; i++) {
                    const auto &etc(sm.etc[ivorder[i]]);
                    for (int j = 0; j < 2; j++)
                    {
                        int qcoord = round(etc(j) * quant);
                        dw.writeDelta(qcoord, last[j]);
                    }
                }
            }
        }

        // write delta coded internal texcoords
        int b2 = dw.nbytes();
        if (flags & SubMeshFlag::internalTexture)
        {
            // TODO: check: ntc must be < 2^16
            int ntc(sm.tc.size());
            bin::write(out, uint16_t(ntc));

            math::Size2 dflt{DefaultTextureSize, DefaultTextureSize};
            auto ts(textureSize(atlas, smIndex, dflt));

            int quant[2] = {
                int(ts.width) << TexCoordSubPixelBits,
                int(ts.height) << TexCoordSubPixelBits
            };
            bin::write(out, uint16_t(quant[0]));
            bin::write(out, uint16_t(quant[1]));

            std::vector<int> itorder;
            invertIndex(torder, itorder);

            int last[2] = {0, 0};
            for (int i = 0; i < ntc; i++) {
                const auto &t(sm.tc[itorder[i]]);
                for (int j = 0; j < 2; j++)
                {
                    int qcoord = round(t(j) * quant[j]);
                    dw.writeDelta(qcoord, last[j]);
                }
            }
        }

        // write faces
        int b3, b4;
        {
            // TODO: check: nf must be < 2^16
            int nf(sm.faces.size());
            bin::write(out, uint16_t(nf));

            // write delta coded vertex indices
            b3 = dw.nbytes();
            for (int i = 0, high = -1; i < nf; i++) {
                const auto &face = sm.faces[forder[i]];
                for (int j = 0; j < 3; j++)
                {
                    int index = vorder[face(j)];
                    dw.writeWord(high+1 - index);
                    if (index > high) { high = index; }
                }
            }

            // write delta coded texcoord indices
            b4 = dw.nbytes();
            if (flags & SubMeshFlag::internalTexture)
            {
                for (int i = 0, high = -1; i < nf; i++) {
                    const auto &face = sm.facesTc[forder[i]];
                    for (int j = 0; j < 3; j++)
                    {
                        int index = torder[face(j)];
                        dw.writeWord(high+1 - index);
                        if (index > high) { high = index; }
                    }
                }
            }
        }
#if 0
        int sm = dw.nsmall(), bg = dw.nbig();

        LOG(info1) << "nsmall = " << sm;
        LOG(info1) << "nbig = " << bg << " (" << double(bg)/(bg+sm)*100 << "%).";

        double total = (dw.nbytes() - b1) / 100;
        int vsize = b2 - b1 - 4, tsize = b3 - b2 - 6;
        int isize1 = b4 - b3, isize2 = dw.nbytes() - b4;

        LOG(info1) << "vertices: " << vsize << " B (" << vsize/total << "%).";
        LOG(info1) << "texcoords: " << tsize << " B (" << tsize/total << "%).";
        LOG(info1) << "v. indices: " << isize1 << " B (" << isize1/total << "%).";
        LOG(info1) << "t. indices: " << isize2 << " B (" << isize2/total << "%).";
#endif
        (void) b1; (void) b2; (void) b3; (void) b4;
    }
}

void saveMeshVersion2(std::ostream &out, const Mesh &mesh)
{
    // helper functions
    auto saveVertexComponent([&out](double v, double o, double s) -> void
    {
        bin::write
            (out, std::uint16_t
             (std::round
              (((v - o) * std::numeric_limits<std::uint16_t>::max()) / s)));
    });

    auto saveTexCoord([&out](double v)
    {
        v = std::round(math::clamp(v, 0.0, 1.0)
                       * std::numeric_limits<std::uint16_t>::max());
        bin::write(out, std::uint16_t(v));
    });

    // write header
    bin::write(out, MAGIC);
    bin::write(out, std::uint16_t(2));

    // no mean undulation
    bin::write(out, double(0.0));

    bin::write(out, std::uint16_t(mesh.submeshes.size()));

    // write submeshes
    for (const auto &sm : mesh) {
        auto bbox(extents(sm));
        math::Point3d bbsize(bbox.ur - bbox.ll);

        // build and write flags
        std::uint8_t flags(0);
        if (!sm.tc.empty()) {
            flags |= SubMeshFlag::internalTexture;
        }
        if (!sm.etc.empty()) {
            flags |= SubMeshFlag::externalTexture;
        }
        if (sm.textureMode == SubMesh::TextureMode::external) {
            flags |= SubMeshFlag::textureMode;
        }
        bin::write(out, flags);

        // surface reference (defaults to 1)
        bin::write(out, std::uint8_t(sm.surfaceReference));

        // write (external) texture layer information
        if (sm.textureLayer) {
            bin::write(out, std::uint16_t(*sm.textureLayer));
        } else {
            // save zero
            bin::write(out, std::uint16_t(0));
        }

        // write extents
        bin::write(out, bbox.ll(0));
        bin::write(out, bbox.ll(1));
        bin::write(out, bbox.ll(2));
        bin::write(out, bbox.ur(0));
        bin::write(out, bbox.ur(1));
        bin::write(out, bbox.ur(2));

        // write sub-mesh data
        bin::write(out, std::uint16_t(sm.vertices.size()));

        auto ietc(sm.etc.begin());
        for (const auto &vertex : sm.vertices) {
            saveVertexComponent(vertex(0), bbox.ll(0), bbsize(0));
            saveVertexComponent(vertex(1), bbox.ll(1), bbsize(1));
            saveVertexComponent(vertex(2), bbox.ll(2), bbsize(2));

            if (flags & SubMeshFlag::externalTexture) {
                saveTexCoord((*ietc)(0));
                saveTexCoord((*ietc)(1));
                ++ietc;
            }
        }

        // save (internal) texture coordinates
        if (flags & SubMeshFlag::internalTexture) {
            bin::write(out, std::uint16_t(sm.tc.size()));
            for (const auto &tc : sm.tc) {
                saveTexCoord(tc(0));
                saveTexCoord(tc(1));
            }
        }

        // save faces
        bin::write(out, std::uint16_t(sm.faces.size()));
        auto ifacesTc(sm.facesTc.begin());

        // TODO: check faces/facesTc indices to be in range 0-2^16-1
        for (auto &face : sm.faces) {
            bin::write(out, std::uint16_t(face(0)));
            bin::write(out, std::uint16_t(face(1)));
            bin::write(out, std::uint16_t(face(2)));

            // save (optional) texture coordinate indices
            if (flags & SubMeshFlag::internalTexture) {
                bin::write(out, std::uint16_t((*ifacesTc)(0)));
                bin::write(out, std::uint16_t((*ifacesTc)(1)));
                bin::write(out, std::uint16_t((*ifacesTc)(2)));
                ++ifacesTc;
            }
        }
    }
}

} // namespace

void saveMeshProper(std::ostream &out, const Mesh &mesh, const Atlas *atlas)
{
    if (NO_MESH_COMPRESSION) {
        saveMeshVersion3(out, mesh, atlas);
    }
    else {
        saveMeshVersion2(out, mesh);
    }
}

} // namespace detail

void saveSubMeshAsObj(std::ostream &out, const SubMesh &sm
                      , std::size_t index, const Atlas*
                      , const std::string &matlib)
{
    out.setf(std::ios::scientific, std::ios::floatfield);

    const bool hasTc(!sm.facesTc.empty());

    if (hasTc && !matlib.empty()) {
        out << "mtllib " << matlib << '\n';
    }

    for (const auto &vertex : sm.vertices) {
        out << "v " << vertex(0) << ' ' << vertex(1) << ' '  << vertex(2)
            << '\n';
    }

    if (hasTc) {
        for (const auto &tc : sm.tc) {
            out << "vt " << tc(0) << ' ' << tc(1) << '\n';
        }
    }

    if (hasTc && !matlib.empty()) {
        out << "usemtl " << index << '\n';
    }

    auto ifacesTc(sm.facesTc.begin());
    for (const auto &face : sm.faces) {
        if (hasTc) {
            const auto &faceTc(*ifacesTc++);
            out << "f " << (face(0) + 1) << '/' << (faceTc(0) + 1)
                << ' ' << (face(1) + 1) << '/' << (faceTc(1) + 1)
                << ' ' << (face(2) + 1) << '/' << (faceTc(2) + 1)
                << '\n';
        } else {
            out << "f " << (face(0) + 1) << ' ' << (face(1) + 1) << ' '
                << (face(2) + 1) << '\n';
        }
    }
}

void saveSubMeshAsObj(const boost::filesystem::path &filepath
                      , const SubMesh &sm, std::size_t index
                      , const Atlas *atlas
                      , const std::string &matlib)
{
    LOG(info2) << "Saving submesh to file <" << filepath << ">.";

    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(filepath.string(), std::ios_base::out | std::ios_base::trunc);
    } catch (const std::exception&) {
        LOGTHROW(err3, std::runtime_error)
            << "Unable to save mesh to <" << filepath << ">.";
    }
    saveSubMeshAsObj(f, sm, index, atlas, matlib);
}

} } // namespace vadstena::vts
