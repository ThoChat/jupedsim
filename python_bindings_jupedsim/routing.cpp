// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionGeometry.hpp"
#include "RoutingEngine.hpp"

#include <glm/ext/vector_float2.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

#include <cstddef>

namespace py = pybind11;

void init_routing(py::module_& m)
{
    py::class_<RoutingEngine>(m, "RoutingEngine")
        .def(py::init([](const CollisionGeometry& geo) {
            return std::make_unique<RoutingEngine>(geo.Polygon());
        }))
        .def("compute_waypoints", &RoutingEngine::ComputeAllWaypoints)
        .def("is_routable", &RoutingEngine::IsRoutable)
        .def("mesh", [](const RoutingEngine& routingEngine) {
            const auto mesh = routingEngine.MeshData();
            const auto verts = mesh->FVertices();
            py::list pyVerts(verts.size());
            for(size_t i = 0; i < verts.size(); ++i) {
                pyVerts[i] = py::make_tuple(verts[i].x, verts[i].y);
            }
            const auto polygonCount = mesh->CountPolygons();
            py::list polys(polygonCount);
            for(size_t index = 0; index < polygonCount; ++index) {
                const auto& poly = mesh->Polygons(index);
                const auto& vertices = poly.vertices;
                py::list pyPoly(vertices.size());
                for(size_t vertIndex = 0; vertIndex < vertices.size(); ++vertIndex) {
                    pyPoly[vertIndex] = vertices[vertIndex];
                }
                polys[index] = pyPoly;
            }
            return py::make_tuple(pyVerts, polys);
        });
}
