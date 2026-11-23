import numpy as np
import pyray as rl

from msgq.visionipc import VisionIpcClient
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.common.constants import CV
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.renderer import EGO, ROUTE_INTENSITY, SIZE
from openpilot.selfdrive.ui.onroad.cameraview import CONNECTION_RETRY_INTERVAL
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, TextAlignmentVertical
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel, gui_label

# The legacy (big) UI renders at 2160x1080, ~3x the 536x240 logical resolution the new
# UI's panel was designed for, so the pixel geometry below is that panel's scaled 3x.
PANEL_WIDTH = 261 * 3
STALE_SECONDS = 3
MARKER_MARGIN = 18  # texture-space pixels kept below the ego so its marker stays visible

# Recolor the model's grayscale frame for people: dim the white roads and make the grey route pop. Only the route's
# exact level (and its neighbors) is recolored, so antialiased road edges passing through that grey stay grey.
MAP_COLORS = np.repeat(np.round(np.arange(256) * 0.45).astype(np.uint8)[:, None], 3, axis=1)
MAP_COLORS[ROUTE_INTENSITY - 1:ROUTE_INTENSITY + 2] = (72, 235, 132)


def format_distance(meters: float) -> str:
  """m / ft under 1000 m / 300 m, km / mi beyond."""
  if ui_state.is_metric:
    return f'{round(meters / 10) * 10} m' if meters < 1000 else f'{meters / 1000:.1f} km'
  return f'{round(meters * CV.METER_TO_FOOT / 50) * 50} ft' if meters < 300 else f'{meters * CV.METER_TO_MILE:.1f} mi'


class NavigationPanel(Widget):
  """Turn instructions around the map frames mapsd publishes for the model."""

  def __init__(self):
    super().__init__()
    self._client = VisionIpcClient('mapsd', VisionStreamType.VISION_STREAM_MAP, True)
    self._last_connect = 0.0
    self._frame_time = 0.0
    self._texture = None
    self._eta_label = UnifiedLabel('', 102, FontWeight.BOLD, rl.WHITE, scroll=True)
    self._accent = rl.Color(155, 230, 185, 255)
    self._muted = rl.Color(162, 175, 185, 255)

  def _update_state(self):
    if not self._client.is_connected():
      if rl.get_time() - self._last_connect < CONNECTION_RETRY_INTERVAL:
        return
      self._last_connect = rl.get_time()
      if not self._client.connect(False) or not self._client.num_buffers:
        return
      self._unload_texture()
      self._texture = rl.load_texture_from_image(rl.Image(None, SIZE, SIZE, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8))
    frame = self._client.recv(timeout_ms=0)
    if frame is not None:
      gray = np.frombuffer(frame.data, dtype=np.uint8, count=SIZE * SIZE)
      rl.update_texture(self._texture, rl.ffi.cast("void *", rl.ffi.from_buffer(MAP_COLORS[gray])))
      # Without a usable map, mapsd publishes black frames with a zero timestamp.
      self._frame_time = rl.get_time() if self._client.timestamp_sof else 0.0

  @property
  def routing(self):
    return ui_state.sm.valid['navInstruction'] and ui_state.sm.alive['navInstruction']

  @property
  def map_ready(self):
    return self._texture is not None and 0 <= rl.get_time() - self._frame_time < STALE_SECONDS

  def _draw_status(self, rect):
    """What the map is waiting on, in place of the map."""
    statuses = (
      ('GPS fix', latest_gps_source(ui_state.sm) is not None),
      ('Route', self.routing),
      ('Map frames', self.map_ready),
    )
    for i, (name, ok) in enumerate(statuses):
      gui_label(rl.Rectangle(rect.x + 30, rect.y + 36 + i * 84, rect.width - 60, 78), f'{name}: {"yes" if ok else "no"}', 66,
                self._accent if ok else self._muted, FontWeight.MEDIUM, alignment_vertical=TextAlignmentVertical.TOP)

  def _render(self, rect):
    x, y = rect.x, rect.y
    rl.draw_rectangle_rec(rect, rl.Color(7, 13, 18, 255))
    instruction = ui_state.sm['navInstruction']
    if self.routing:
      direction = instruction.maneuverModifier
      flip = -1 if 'left' in direction else 1
      # The maneuver arrow on a 4.2x grid, centered in the header's icon box, with round joins for the thick stroke.
      if direction in ('straight', ''):
        shape = [(0, 14), (0, -12), (-6, -5), (0, -12), (6, -5)]
      else:
        shape = [(-flip*9, 14), (-flip*9, -4), (-flip*2, -9), (flip*9, -9), (flip*3, -15), (flip*9, -9), (flip*3, -3)]
      points = [rl.Vector2(x + 93 + 4.2*px, y + 90 + 4.2*py) for px, py in shape]
      for a, b in zip(points[:-1], points[1:], strict=True):
        rl.draw_line_ex(a, b, 21, self._accent)
      for point in points:
        rl.draw_circle_v(point, 10.5, self._accent)
      gui_label(rl.Rectangle(x+192, y+33, rect.width-216, 120), format_distance(max(0, instruction.maneuverDistance)), 102, rl.WHITE, FontWeight.BOLD,
                alignment_vertical=TextAlignmentVertical.TOP)
    else:
      gui_label(rl.Rectangle(x+36, y+54, rect.width-72, 90), 'Waiting for route', 78, rl.WHITE, FontWeight.BOLD,
                alignment_vertical=TextAlignmentVertical.TOP)
    map_rect = rl.Rectangle(x+24, y+183, rect.width-48, rect.height-333)
    rl.draw_rectangle_rec(map_rect, rl.BLACK)
    if self.map_ready:
      # Crop the model's frame with the map rect's aspect, centered horizontally with the ego near the bottom,
      # then upscale to fill the map rect.
      aspect = map_rect.width / map_rect.height
      if aspect >= 1.0:
        width = SIZE
        height = max(1, int(SIZE / aspect))
      else:
        height = SIZE
        width = max(1, int(SIZE * aspect))
      source = rl.Rectangle((SIZE - width) // 2, min(SIZE - height, max(0, int(EGO[1] + MARKER_MARGIN - height))), width, height)
      rl.draw_texture_pro(self._texture, source, map_rect, rl.Vector2(0, 0), 0, rl.WHITE)
    else:
      self._draw_status(map_rect)
    if self.routing:
      seconds = max(0, instruction.timeRemaining)
      remaining = max(0, instruction.distanceRemaining)
      self._eta_label.set_text(f'{max(1, round(seconds / 60))} min / {format_distance(remaining)}')
      self._eta_label.render(rl.Rectangle(x+27, y+rect.height-135, rect.width-54, 120))

  def _unload_texture(self):
    if self._texture:
      rl.unload_texture(self._texture)
      self._texture = None

  def close(self):
    self._unload_texture()
