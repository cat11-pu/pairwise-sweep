"""Axis aligned bounding box collision queries and sub-stepped movement.

The module is dependency free and only uses the standard library.  Boxes are
stored as a centre point plus half extents, which keeps the queries cheap for
the sprite sizes this engine works with.
"""

import math

__all__ = [
    "AABB",
    "Hit",
    "aabb_from_ltrb",
    "axis_times",
    "sweep_aabb",
    "substep_count",
    "move_and_collide",
]

MAX_RESOLVE_ITERS = 8


class AABB:
    """Axis aligned box described by its centre and half extents."""

    __slots__ = ("cx", "cy", "hw", "hh")

    def __init__(self, cx, cy, hw, hh):
        cx = float(cx)
        cy = float(cy)
        hw = float(hw)
        hh = float(hh)
        if hw <= 0.0 or hh <= 0.0:
            raise ValueError("half extents must be positive")
        self.cx = cx
        self.cy = cy
        self.hw = hw
        self.hh = hh

    @property
    def left(self):
        return self.cx - self.hw

    @property
    def right(self):
        return self.cx + self.hw

    @property
    def bottom(self):
        return self.cy - self.hh

    @property
    def top(self):
        return self.cy + self.hh

    @property
    def width(self):
        return 2.0 * self.hw

    @property
    def height(self):
        return 2.0 * self.hh

    def moved(self, dx, dy):
        """A copy of this box shifted by the given offset."""
        return AABB(self.cx + dx, self.cy + dy, self.hw, self.hh)

    def expanded(self, amount):
        """A copy of this box used for queries over a wider area."""
        return AABB(self.cx + amount, self.cy + amount, self.hw, self.hh)

    def overlaps(self, other):
        """True when the interiors of both boxes share space."""
        return (
            self.left < other.right
            and self.right > other.left
            and self.bottom <= other.top
            and self.top > other.bottom
        )

    def __repr__(self):
        return "AABB(cx=%r, cy=%r, hw=%r, hh=%r)" % (self.cx, self.cy, self.hw, self.hh)


class Hit:
    """A contact produced by a swept query or by a separation step."""

    __slots__ = ("t", "nx", "ny", "depth", "box", "other")

    def __init__(self, t, nx, ny, depth, box, other):
        self.t = t
        self.nx = nx
        self.ny = ny
        self.depth = depth
        self.box = box
        self.other = other

    def __repr__(self):
        return "Hit(t=%r, nx=%r, ny=%r, depth=%r)" % (self.t, self.nx, self.ny, self.depth)


def aabb_from_ltrb(left, bottom, right, top):
    """Build a box from its left, bottom, right and top edges."""
    left = float(left)
    bottom = float(bottom)
    right = float(right)
    top = float(top)
    if right < left or top < bottom:
        raise ValueError("rectangle edges must not be inverted")
    return AABB(
        (left + right) * 0.5,
        (bottom + top) * 0.5,
        (right - left) * 0.5,
        (top - bottom) * 0.5,
    )


def axis_times(lo, hi, d, olo, ohi):
    """Fractions of a move during which two intervals share space.

    lo and hi bound the moving interval and d is its displacement over the
    whole step, while olo and ohi bound the fixed interval.  The result is the
    pair (entered, left), or None when the intervals never share space.
    """
    if d == 0.0:
        if hi <= olo or lo >= ohi:
            return None
        return (0.0, 1.0)
    inv = 1.0 / d
    t1 = (olo - hi) * inv
    t2 = (ohi - lo) * inv
    return (t1, t2)


def _face_x(box, other):
    return -1.0 if box.cx <= other.cx else 1.0


def _face_y(box, other):
    return -1.0 if box.cy <= other.cy else 1.0


def _blend_normals(a, b):
    """Unit normal half way between two perpendicular face normals."""
    x = a[0] + b[0]
    y = a[1] + b[1]
    norm = x * x + y * y
    if norm > 0.0:
        norm = 1.0 / norm
        x = x * norm
        y = y * norm
    return (x, y)


def sweep_aabb(box, dx, dy, obstacles):
    """Earliest contact of a moving box with any obstacle during one step.

    Returns the first Hit whose fraction t lies in [0, 1] and whose normal is a
    unit vector pointing away from the obstacle, or None when the movement stays
    clear of every obstacle.  A move that only grazes a corner does not count as
    a contact, and a box that already shares space with an obstacle is left to
    the separation step instead.
    """
    best = None
    for other in obstacles:
        if box.overlaps(other):
            continue
        tx = axis_times(box.left, box.right, dx, other.left, other.right)
        if tx is None:
            continue
        ty = axis_times(box.bottom, box.top, dy, other.bottom, other.top)
        if ty is None:
            continue
        enter = tx[0] if tx[0] > ty[0] else ty[0]
        leave = tx[1] if tx[1] < ty[1] else ty[1]
        if enter >= leave:
            continue
        if enter <= 0.0:
            continue
        if enter > 1.0:
            continue
        if abs(dx) > abs(dy):
            nx, ny = _face_x(box, other), 0.0
        elif abs(dy) > abs(dx):
            nx, ny = 0.0, _face_y(box, other)
        else:
            nx, ny = _blend_normals((_face_x(box, other), 0.0), (0.0, _face_y(box, other)))
        moved = box.moved(dx * enter, dy * enter)
        hit = Hit(enter, nx, ny, 0.0, moved, other)
        if best is None or hit.t < best.t:
            best = hit
    return best


def substep_count(box, dx, dy, obstacles, max_step=1.0):
    """Number of equal substeps a move is split into.

    Every substep is at most max_step long and at most as long as the smallest
    dimension of the body or of any obstacle, so that no contact can be skipped.
    """
    span = max(abs(dx), abs(dy))
    if span <= 0.0:
        return 1
    limit = float(max_step)
    count = int(math.ceil(span / limit))
    return count if count > 1 else 1


def _first_overlap(box, obstacles):
    for other in obstacles:
        if box.overlaps(other):
            return other
    return None


def _separate(box, other, vx, vy):
    """Push a body out of an obstacle it shares space with.

    Returns (body, hit) with the body placed clear of the obstacle, or None
    when the two boxes do not overlap.
    """
    ox = (box.hw + other.hw) - abs(box.cx - other.cx)
    oy = (box.hh + other.hh) - abs(box.cy - other.cy)
    if ox <= 0.0 or oy <= 0.0:
        return None
    if ox < oy:
        nx = -1.0 if vx > 0.0 else 1.0
        ny = 0.0
        depth = ox
    else:
        nx = 0.0
        ny = -1.0 if vy > 0.0 else 1.0
        depth = oy
    moved = box.moved(nx * depth, ny * depth)
    return moved, Hit(0.0, nx, ny, depth, moved, other)


def move_and_collide(box, dx, dy, obstacles, max_step=1.0):
    """Move a body by (dx, dy), stopping where obstacles block the way.

    Returns the moved body together with the contacts that were resolved, in
    order.  The body never ends the move inside an obstacle, and it travels
    exactly (dx, dy) when nothing blocks it.
    """
    steps = substep_count(box, dx, dy, obstacles, max_step)
    step_x = dx / steps
    step_y = dy / steps
    body = box
    hits = []
    for _ in range(steps):
        body = body.moved(step_x, step_y)
        for _ in range(MAX_RESOLVE_ITERS):
            other = _first_overlap(body, obstacles)
            if other is None:
                break
            separated = _separate(body, other, step_x, step_y)
            if separated is None:
                break
            body, hit = separated
            hits.append(hit)
    return body, hits
