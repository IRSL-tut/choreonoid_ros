/*!
  @author Shin'ichiro Nakaoka
*/

#include "../deprecated/BodyPublisherItem.h"
#include "../WorldROSItem.h"
#include "../ROSControlItem.h"
#include "../BodyROSItem.h"

#include <cnoid/PyUtil>
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(ROSPlugin, m)
{
    m.doc() = "Choreonoid ROSPlugin module";

    py::module::import("cnoid.Base");
    py::module::import("cnoid.BodyPlugin");

    py::class_<BodyPublisherItem, BodyPublisherItemPtr, ControllerItem>(m, "BodyPublisherItem")
        .def(py::init<>())
        ;

    py::class_<WorldROSItem, WorldROSItemPtr, Item>(m, "WorldROSItem")
        .def(py::init<>())
        .def_property("MaxClockPublishingRate",
                      &WorldROSItem::getMaxClockPublishingRate,
                      &WorldROSItem::setMaxClockPublishingRate)
        ;

    py::class_<ROSControlItem, ROSControlItemPtr, ControllerItem>(m, "ROSControlItem")
        .def(py::init<>())
        .def_property("NameSpace", &ROSControlItem::getNameSpace, &ROSControlItem::setNameSpace)
        ;

    py::class_<BodyROSItem, BodyROSItemPtr, ControllerItem>(m, "BodyROSItem")
        .def(py::init<>())
        .def_property_readonly("controlTime", &BodyROSItem::controlTime)
        .def_property("NameSpace", &BodyROSItem::getNameSpace, &BodyROSItem::setNameSpace)
        .def_property("JointStateUpdateRate",
                      &BodyROSItem::getJointStateUpdateRate,
                      &BodyROSItem::setJointStateUpdateRate)
        ;
}


