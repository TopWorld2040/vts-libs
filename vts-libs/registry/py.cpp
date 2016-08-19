#include "pysupport/string.hpp"

#include "./py.hpp"

namespace python = boost::python;

namespace vadstena { namespace registry {

using pysupport::py2utf8;

namespace {

void parseIntSet(std::set<int> &set, const python::list &value)
{
    for (python::stl_input_iterator<int> ivalue(value), evalue;
         ivalue != evalue; ++ivalue)
    {
        set.insert(*ivalue);
    }
}

} // namespace

void fromPython(BoundLayer &bl, const python::dict &value)
{
    if (value.has_key("id")) {
        bl.numericId = python::extract<int>(value["id"]);
    }

    bl.type = boost::lexical_cast<BoundLayer::Type>
        (py2utf8(value["type"]));

    bl.url = py2utf8(value["url"]);
    if (value.has_key("maskUrl")) {
        bl.maskUrl = py2utf8(value["maskUrl"]);
    }
    if (value.has_key("metaUrl")) {
        bl.metaUrl = py2utf8(value["metaUrl"]);
    }

    const auto lodRange(value["lodRange"]);
    bl.lodRange.min = python::extract<Lod>(lodRange[0]);
    bl.lodRange.max = python::extract<Lod>(lodRange[1]);

    const auto tileRange(value["tileRange"]);
    const auto tileRange_min(tileRange[0]);
    bl.tileRange.ll(0) = python::extract<unsigned int>(tileRange_min[0]);
    bl.tileRange.ll(1) = python::extract<unsigned int>(tileRange_min[1]);

    const auto tileRange_max(tileRange[1]);
    bl.tileRange.ur(0) = python::extract<unsigned int>(tileRange_max[0]);
    bl.tileRange.ur(1) = python::extract<unsigned int>(tileRange_max[1]);

    // parse credits
    {
        const auto cr(value["credits"]);
        python::extract<std::string> string(cr);
        if (string.check()) {
            // url
            bl.creditsUrl = py2utf8(cr);
        } else {
            // credits
            fromPython(bl.credits, python::dict(cr));
        }
    }

    if (value.has_key("availability")) {
        python::dict availability(value[availability]);

        bl.availability = boost::in_place();
        auto &bla(*bl.availability);

        bla.type = boost::lexical_cast<BoundLayer::Availability::Type>
            (py2utf8(availability["type"]));

        switch (bla.type) {
        case BoundLayer::Availability::Type::negativeType:
            bla.mime = py2utf8(availability["mime"]);
            break;

        case BoundLayer::Availability::Type::negativeCode:
            parseIntSet(bla.codes, python::list(availability["codes"]));
            break;

        case BoundLayer::Availability::Type::negativeSize:
            bla.size = python::extract<int>(availability["size"]);
            break;
        }
    }

    // isTransparent is optional
    if (value.has_key("isTransparent")) {
        bl.isTransparent = python::extract<bool>(value["isTransparent"]);
    }
}

void fromPython(BoundLayer::dict &boundLayers
                , const python::dict &value)
{
    for (python::stl_input_iterator<python::str> ivalue(value), evalue;
         ivalue != evalue; ++ivalue)
    {
        const auto id(py2utf8(*ivalue));
        python::dict content(value[*ivalue]);

        BoundLayer bl;
        bl.id = id;
        if (python::extract<python::str>(content).check()) {
            // special case: external url
            bl.type = BoundLayer::Type::external;
            bl.url = py2utf8(content);
        } else {
            fromPython(bl, content);
        }
        boundLayers.add(bl);
    }
}

void fromPython(Credit &credit, const python::dict &value)
{
    credit.numericId = python::extract<int>(value["id"]);
    credit.notice = py2utf8(value["notice"]);

    if (value.has_key("url")) {
        credit.url = py2utf8(value["url"]);
    }

    if (value.has_key("copyrighted")) {
        credit.copyrighted = python::extract<bool>(value["copyrighted"]);
    } else {
        credit.copyrighted = true;
    }
}

void fromPython(Credit::dict &credits, const python::dict &value)
{
    for (python::stl_input_iterator<python::str> ivalue(value), evalue;
         ivalue != evalue; ++ivalue)
    {
        const auto id(py2utf8(*ivalue));
        python::dict content(value[*ivalue]);
        Credit c;
        c.id = id;
        fromPython(c, content);
        credits.add(c);
    }
}

void fromPython(RegistryBase &rb, const python::dict &value)
{
    // parse value
    if (value.has_key("credits")) {
        fromPython(rb.credits, python::dict(value["credits"]));
    }
    if (value.has_key("boundLayers")) {
        fromPython(rb.boundLayers, python::dict(value["boundLayers"]));
    }
}

void fromPython(Credits &credits, const python::object &value)
{
    python::extract<python::list> list(value);
    if (list.check()) {
        for (python::stl_input_iterator<python::object> ivalue(value), evalue;
             ivalue != evalue; ++ivalue)
        {
            credits.set(py2utf8(*ivalue), boost::none);
        }
        return;
    }

    python::extract<python::dict> dict(value);
    if (dict.check()) {
        for (python::stl_input_iterator<python::str> ivalue(value), evalue;
             ivalue != evalue; ++ivalue)
        {
            const auto id(py2utf8(*ivalue));
            python::dict element(value[*ivalue]);

            if (!len(element)) {
                credits.set(id, boost::none);
            } else {
                Credit c;
                c.id = id;
                fromPython(c, element);
                credits.add(c);
            }
        }
        return;
    }

    LOGTHROW(err1, std::runtime_error)
        << "Type of credits is not a (python) list nor an (python) object.";
}

} } // namespace vadstena::registry
