# SPDX-License-Identifier: LGPL-3.0-or-later
from jupedsim.internal.aabb import AABB
from jupedsim.recording import Recording, RecordingFrame
from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkPolyData
from vtkmodules.vtkFiltersCore import vtkGlyph2D
from vtkmodules.vtkFiltersSources import vtkRegularPolygonSource
from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper

from jupedsim_visualizer.config import Colors, ZLayers

GS_SCALING_FACTOR = 0.26 / (2 * 0.3 * 1.65)
DEFAULT_AGENT_RADIUS = 0.15
DEFAULT_AGENT_HEIGHT = 1.65


def to_vtk_points(frame: RecordingFrame) -> vtkPoints:
    points = vtkPoints()
    for agent in frame.agents:
        points.InsertNextPoint(
            agent.position[0], agent.position[1], ZLayers.agents
        )
    return points


def _ground_support_radius(agent) -> float:
    height = (
        agent.height if agent.height is not None else DEFAULT_AGENT_HEIGHT
    )
    radius = (
        agent.radius if agent.radius is not None else DEFAULT_AGENT_RADIUS
    )
    return radius * GS_SCALING_FACTOR * height


def build_link_polydata(frame: RecordingFrame) -> vtkPolyData:
    from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkLine

    points = vtkPoints()
    lines = vtkCellArray()
    for agent in frame.agents:
        if agent.ground_support_position is not None:
            idx = points.InsertNextPoint(
                agent.position[0], agent.position[1], ZLayers.agents
            )
            idx2 = points.InsertNextPoint(
                agent.ground_support_position[0],
                agent.ground_support_position[1],
                ZLayers.agents,
            )
            line = vtkLine()
            line.GetPointIds().SetId(0, idx)
            line.GetPointIds().SetId(1, idx2)
            lines.InsertNextCell(line)
    polydata = vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetLines(lines)
    return polydata


def clamp(value: int, min_value: int, max_value: int) -> int:
    return max(min_value, min(max_value, value))


def _velocity_magnitude(velocity: tuple[float, float] | None) -> float:
    if velocity is None:
        return 0.0
    return (velocity[0] ** 2 + velocity[1] ** 2) ** 0.5


def _mix_colors(
    start: list[float], end: list[float], factor: float
) -> list[float]:
    return [s + (e - s) * factor for s, e in zip(start, end)]


def _debug_color(value: float, max_value: float) -> list[float]:
    low_color = [0.86, 0.24, 0.12]
    mid_color = [0.00, 0.68, 0.68]
    high_color = [0.10, 0.36, 0.62]
    if max_value <= 0:
        return low_color
    clamped = max(0.0, min(1.0, value / max_value))
    if clamped <= 0.5:
        return _mix_colors(low_color, mid_color, clamped * 2.0)
    return _mix_colors(mid_color, high_color, (clamped - 0.5) * 2.0)


class Trajectory:
    def __init__(self, rec: Recording) -> None:
        self.rec = rec
        self.current_index = 0
        self.num_frames = rec.num_frames
        self._has_ipp = rec.has_ipp_columns

        first_frame = rec.frame(0)

        polydata = vtkPolyData()
        polydata.SetPoints(to_vtk_points(first_frame))
        self.polydata = polydata

        polygon_source = vtkRegularPolygonSource()
        polygon_source.SetRadius(0.15)
        polygon_source.SetNumberOfSides(24)

        glyph2d = vtkGlyph2D()
        glyph2d.SetSourceConnection(polygon_source.GetOutputPort())
        glyph2d.SetInputData(polydata)
        glyph2d.Update()
        self.glyph2D = glyph2d

        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(glyph2d.GetOutputPort())
        mapper.Update()

        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(Colors.agent)
        self.actor = actor

        self.gs_actors = []
        self.gs_sources = []
        self.link_actor = None
        self._link_mapper = None

        if self._has_ipp:
            self._init_ipp(first_frame)

    def _init_ipp(self, first_frame: RecordingFrame):
        ground_support_agents = [
            agent
            for agent in first_frame.agents
            if agent.ground_support_position is not None
        ]
        for agent in ground_support_agents:
            source = vtkRegularPolygonSource()
            source.SetNumberOfSides(24)
            source.SetCenter(
                agent.ground_support_position[0],
                agent.ground_support_position[1],
                ZLayers.agents,
            )
            source.SetRadius(_ground_support_radius(agent))

            mapper = vtkPolyDataMapper()
            mapper.SetInputConnection(source.GetOutputPort())

            actor = vtkActor()
            actor.SetMapper(mapper)

            self.gs_sources.append(source)
            self.gs_actors.append(actor)

        self._update_ground_support_actors(first_frame)

        link_mapper = vtkPolyDataMapper()
        link_mapper.SetInputData(build_link_polydata(first_frame))
        self._link_mapper = link_mapper

        link_actor = vtkActor()
        link_actor.SetMapper(link_mapper)
        link_actor.GetProperty().SetColor(0.0, 0.0, 0.0)
        link_actor.GetProperty().SetLineWidth(1.5)
        self.link_actor = link_actor

    def _update_ground_support_actors(self, frame: RecordingFrame):
        ground_support_agents = [
            agent
            for agent in frame.agents
            if agent.ground_support_position is not None
        ]
        max_speed = max(
            (_velocity_magnitude(agent.ground_support_velocity) for agent in ground_support_agents),
            default=0.0,
        )

        for index, actor in enumerate(self.gs_actors):
            if index >= len(ground_support_agents):
                actor.SetVisibility(False)
                continue

            agent = ground_support_agents[index]
            source = self.gs_sources[index]
            source.SetCenter(
                agent.ground_support_position[0],
                agent.ground_support_position[1],
                ZLayers.agents,
            )
            source.SetRadius(_ground_support_radius(agent))
            color = _debug_color(
                _velocity_magnitude(agent.ground_support_velocity), max_speed
            )
            actor.GetProperty().SetColor(color)
            actor.SetVisibility(True)
            source.Update()

    def get_actors(self) -> list[vtkActor]:
        actors = [self.actor]
        actors.extend(self.gs_actors)
        if self.link_actor is not None:
            actors.append(self.link_actor)
        return actors

    def get_bounds(self) -> AABB:
        return self.rec.bounds()

    def _update_frame(self, frame: RecordingFrame):
        self.polydata.SetPoints(to_vtk_points(frame))
        self.glyph2D.Update()

        if self._has_ipp:
            self._update_ground_support_actors(frame)
            self._link_mapper.SetInputData(build_link_polydata(frame))

    def advance_frame(self, offset: int):
        self.current_index = clamp(
            self.current_index + offset, 0, self.num_frames - 1
        )
        self._update_frame(self.rec.frame(self.current_index))

    def goto_frame(self, index: int):
        self.current_index = clamp(index, 0, self.num_frames - 1)
        self._update_frame(self.rec.frame(self.current_index))
