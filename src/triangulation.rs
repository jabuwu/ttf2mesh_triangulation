use std::{
    cmp::Ordering,
    mem::{swap, transmute},
    pin::Pin,
    ptr::NonNull,
};

use snafu::Snafu;

use crate::{linked_list::LinkedListNode, vec2::Vec2};

const EPSILON: f32 = 1e-7;

#[derive(Default)]
pub struct Triangulator {
    contours: Vec<Contour>,
    total_points: usize,
}

impl Triangulator {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_contour(
        &mut self,
        identifier: usize,
        points: Vec<[f32; 2]>,
    ) -> Result<(), TriangulatorError> {
        if points.len() < 3 {
            return Err(TriangulatorError::Incomplete);
        }

        self.total_points += points.len();
        self.contours.push(Contour {
            identifier,
            points: unsafe { transmute(points) },
            is_hole: false,
            nested_to: None,
        });

        for contour_index in 0..self.contours.len() {
            let mut nested_to = None;
            let is_hole = !ttf_outline_contour_info_majority(
                self,
                self.contours[contour_index].identifier,
                contour_index,
                &mut nested_to,
            );
            self.contours[contour_index].nested_to = nested_to;
            self.contours[contour_index].is_hole = is_hole;
        }

        Ok(())
    }

    pub fn triangulate(&self) -> Result<Vec<[[f32; 2]; 3]>, TriangulatorError> {
        if !self.contours.is_empty() {
            let mut mesher = Mesher::new(self);
            mesher.process(128, true, true)?;
            Ok(unsafe { transmute(mesher.triangles()) })
        } else {
            Err(TriangulatorError::Incomplete)
        }
    }
}

struct Contour {
    points: Vec<Vec2>,
    identifier: usize,
    is_hole: bool,
    nested_to: Option<usize>,
}

fn ttf_outline_contour_info_majority(
    outline: &Triangulator,
    identifier: usize,
    contour: usize,
    nested_to: &mut Option<usize>,
) -> bool {
    let mut cont = [false; 3];
    let mut nto: [Option<usize>; 3] = [None; 3];
    let step = outline.contours[contour].points.len() / 3;
    cont[0] = ttf_outline_contour_info(outline, identifier, contour, step * 0, &mut nto[0]);
    cont[1] = ttf_outline_contour_info(outline, identifier, contour, step * 1, &mut nto[1]);
    cont[2] = ttf_outline_contour_info(outline, identifier, contour, step * 2, &mut nto[2]);
    let mut index = None;
    if nto[0].is_none() {
        index = Some(0)
    };
    if nto[1].is_none() {
        index = Some(1)
    };
    if nto[2].is_none() {
        index = Some(2)
    };
    if index.is_none() {
        index = if nto[0] == nto[1] || nto[0] == nto[2] {
            Some(0)
        } else {
            Some(1)
        };
    }
    *nested_to = nto[index.unwrap()];
    return cont[index.unwrap()];
}

fn ttf_outline_contour_info(
    outline: &Triangulator,
    identifier: usize,
    contour: usize,
    test_point: usize,
    nested_to: &mut Option<usize>,
) -> bool {
    let mut count = 0;
    let mut nto = None;
    let mut closest = 0.;
    for i in 0..outline.contours.len() {
        if i == contour {
            continue;
        }
        if outline.contours[i].identifier != identifier {
            continue;
        }
        let mut dist = 0.;
        let res = ttf_outline_evenodd_base(
            outline,
            &outline.contours[contour].points[test_point],
            i,
            &mut dist,
        );
        count += res;
        if (res & 1) == 0 {
            continue;
        };
        if nto.is_none() || dist < closest {
            closest = dist;
            nto = Some(i);
        }
    }
    *nested_to = nto;
    return (count & 1) == 0;
}

fn ttf_outline_evenodd_base(
    outline: &Triangulator,
    point: &Vec2,
    contour: usize,
    dist: &mut f32,
) -> i32 {
    let mut counter = 0;
    let mut closest_dx = 0.;

    let points = &outline.contours[contour].points;
    let len = outline.contours[contour].points.len();

    let mut prev = &points[len - 1];
    for pt in points.iter() {
        let (u, b): (&Vec2, &Vec2);

        if pt.y > prev.y {
            u = pt;
            b = prev;
        } else {
            u = prev;
            b = pt;
        }

        if point.y <= u.y && point.y > b.y {
            if point.x >= u.x || point.x >= b.x {
                //            u
                //      edge /
                //          / <-- dx --> .
                //         /            point
                //        b
                // calculation of distance from edge to point:
                // y(x) = b.y + (u.y - b.y) / (u.x - b.x) * (x - b.x)    - the edge line equation
                // x(y) = (y - b.y) / (u.y - b.y) * (u.x - b.x) + b.x    - inverse equation of line
                // dx = point.x - x(point.y)                             - final equation for distance
                let dy = u.y - b.y;
                if dy.abs() > EPSILON
                // horizontal edges are not handling
                {
                    let dx = point.x - (point.y - b.y) / dy * (u.x - b.x) - b.x;
                    if dx >= 0. {
                        if counter == 0 || dx < closest_dx {
                            closest_dx = dx;
                        }
                        counter += 1;
                    }
                }
            }
        }
        prev = pt;
    }

    *dist = closest_dx;

    counter
}

struct Vertex {
    pos: Vec2,
    contour: usize,
    is_hole: bool,
    nested_to: Option<usize>,
    object: usize,
    edges: VertexToEdgeNode,
    next_in_contour: *mut Vertex,
    prev_in_contour: *mut Vertex,
}

impl Default for Vertex {
    fn default() -> Self {
        Self {
            pos: Vec2::ZERO,
            contour: 0,
            is_hole: false,
            nested_to: None,
            object: 0,
            edges: VertexToEdgeNode::default(),
            next_in_contour: std::ptr::null_mut(),
            prev_in_contour: std::ptr::null_mut(),
        }
    }
}

#[derive(Clone, Copy)]
struct Edge {
    v1: *mut Vertex,
    v2: *mut Vertex,
    tr: [*mut TriangleNode; 2],
    alt_cc: [Circumcircle; 2],
}

impl Default for Edge {
    fn default() -> Self {
        Self {
            v1: std::ptr::null_mut(),
            v2: std::ptr::null_mut(),
            tr: [std::ptr::null_mut(); 2],
            alt_cc: [Circumcircle::default(); 2],
        }
    }
}

type EdgeNode = LinkedListNode<Edge>;

impl Edge {
    fn has_vert(&self, v: *mut Vertex) -> bool {
        self.v1 == v || self.v2 == v
    }

    fn connected(&self, other: *mut EdgeNode) -> bool {
        unsafe { self.has_vert((*other).v1) || self.has_vert((*other).v2) }
    }

    fn second_vert(&self, v: *mut Vertex) -> *mut Vertex {
        if self.v1 == v {
            self.v2
        } else {
            self.v1
        }
    }

    fn common_vert(&self, other: *mut EdgeNode) -> *mut Vertex {
        unsafe {
            if self.v1 == (*other).v1 || self.v1 == (*other).v2 {
                self.v1
            } else {
                self.v2
            }
        }
    }

    fn is_contour_edge(&self) -> bool {
        unsafe { (*self.v1).prev_in_contour == self.v2 || (*self.v2).prev_in_contour == self.v1 }
    }
}

#[derive(Clone, Copy)]
struct Triangle {
    edge: [*mut EdgeNode; 3],
    cc: Circumcircle,
    helper: i32,
}

impl Default for Triangle {
    fn default() -> Self {
        Self {
            edge: [std::ptr::null_mut(); 3],
            cc: Circumcircle::default(),
            helper: 0,
        }
    }
}

impl Triangle {
    fn second_edge(&self, first: *mut EdgeNode) -> *mut EdgeNode {
        if self.edge[0] == first {
            self.edge[1]
        } else {
            self.edge[0]
        }
    }

    fn third_edge(&self, first: *mut EdgeNode) -> *mut EdgeNode {
        if self.edge[2] == first {
            self.edge[1]
        } else {
            self.edge[2]
        }
    }

    fn opposite_vert(&self, edge: *mut EdgeNode) -> *mut Vertex {
        unsafe { (*self.second_edge(edge)).common_vert(self.third_edge(edge)) }
    }

    fn opposite_edge(&self, vertex: *mut Vertex) -> *mut EdgeNode {
        unsafe {
            if (*self.edge[0]).has_vert(vertex) {
                if (*self.edge[1]).has_vert(vertex) {
                    self.edge[2]
                } else {
                    self.edge[1]
                }
            } else {
                self.edge[0]
            }
        }
    }
}

type TriangleNode = LinkedListNode<Triangle>;

struct VertexToEdge {
    edge: *mut EdgeNode,
}

impl Default for VertexToEdge {
    fn default() -> Self {
        Self {
            edge: std::ptr::null_mut(),
        }
    }
}

type VertexToEdgeNode = LinkedListNode<VertexToEdge>;

#[derive(Clone, Copy)]
struct Circumcircle {
    radius: f32,
    center: Vec2,
}

impl Default for Circumcircle {
    fn default() -> Self {
        Self {
            radius: 0.,
            center: Vec2::ZERO,
        }
    }
}

/// Cramer's rule for 2x2 system
macro_rules! cramer2X2 {
    ($x:expr, $y:expr, $a11:expr, $a12:expr, $c1:expr, $a21:expr, $a22:expr, $c2:expr) => {{
        let det = ($a11) * ($a22) - ($a12) * ($a21);
        $x = (($c1) * ($a22) - ($c2) * ($a12)) / det;
        $y = (($c2) * ($a11) - ($c1) * ($a21)) / det;
    }};
}

struct Mesher {
    /// Actual number of vertices (may be less than maxv)
    nv: usize,
    /// Vertex pool, length of maxv
    v: Pin<Vec<Vertex>>,
    /// Edge pool, length of maxe
    _e: Pin<Vec<EdgeNode>>,
    /// Triangle pool, length of maxt
    _t: Pin<Vec<TriangleNode>>,
    /// Vertex->edge pool, length of (2 * maxe)
    _l: Pin<Vec<VertexToEdgeNode>>,
    /// Sorted by (y) array of vertices
    s: Pin<Vec<*mut Vertex>>,
    pinned: Pin<Box<MesherPinned>>,
}

struct MesherPinned {
    /// Root of the list of free edges
    efree: EdgeNode,
    /// Root of the list of edges used
    eused: EdgeNode,
    /// Root of the list of free triangles
    tfree: TriangleNode,
    /// Root of the list of triangles used
    tused: TriangleNode,
    /// Root of free reference list vertex->edge
    lfree: VertexToEdgeNode,
    /// Two initialisation points along the lower edge of the glyph
    vinit: [Vertex; 2],
}

impl Mesher {
    fn new(o: &Triangulator) -> Self {
        // Allocate memory and initialize fields

        let maxv = o.total_points;
        let maxt = ((maxv + 2) - 3) * 2 + 1;
        let maxe = maxt * 2 + 1;
        let maxv2e = maxe * 2;

        let mut v: Vec<Vertex> = Vec::new();
        v.reserve(maxv);

        let mut e: Vec<EdgeNode> = Vec::new();
        e.reserve(maxe);

        let mut t: Vec<TriangleNode> = Vec::new();
        t.reserve(maxt);

        let mut l: Vec<VertexToEdgeNode> = Vec::new();
        l.reserve(maxv2e);

        let mut s: Vec<*mut Vertex> = Vec::new();
        s.reserve(maxv);

        // Filling in the vertices according to outline points
        for i in 0..o.contours.len() {
            let pt = &o.contours[i].points;
            let len = pt.len();
            let base = v.len();
            if len < 3 {
                continue;
            }
            // Byte-by-byte contour matching is done to combat repetition.
            // Designers sometimes allow. Example U+2592 in multiple fonts.
            let mut duplicated = false;
            let mut j = 0;
            while j < i && !duplicated {
                duplicated = compare_contours(o, i, j);
                j += 1;
            }
            if duplicated {
                continue;
            }

            // Define the type of contour (normal or hole) and if hole, who is the parent
            let nested_to = o.contours[i].nested_to;
            let is_hole = o.contours[i].is_hole;

            for j in 0..len {
                let vertex = Vertex {
                    pos: Vec2::new(pt[j].x, pt[j].y),
                    contour: i,
                    is_hole,
                    nested_to,
                    edges: VertexToEdgeNode::default(),
                    prev_in_contour: unsafe {
                        v.as_mut_ptr().offset((base + (j + len - 1) % len) as isize)
                    },
                    next_in_contour: unsafe {
                        v.as_mut_ptr().offset((base + (j + 1) % len) as isize)
                    },
                    ..Default::default()
                };
                v.push(vertex);
                let v_len = v.len();
                s.push(&mut v[v_len - 1]);
                v[v_len - 1].edges.init();
            }
        }

        let nv = v.len();

        // Sort the vertex array by y-coordinate
        s.sort_by(|a, b| unsafe {
            let v1 = (**a).pos;
            let v2 = (**b).pos;
            if v1.y == v2.y {
                if v1.x == v2.x {
                    Ordering::Equal
                } else if v1.x > v2.x {
                    Ordering::Greater
                } else {
                    Ordering::Less
                }
            } else {
                if v1.y > v2.y {
                    Ordering::Greater
                } else {
                    Ordering::Less
                }
            }
        });

        let mut pinned = Pin::new(Box::new(MesherPinned {
            efree: EdgeNode::default(),
            eused: EdgeNode::default(),
            tfree: TriangleNode::default(),
            tused: TriangleNode::default(),
            lfree: VertexToEdgeNode::default(),
            vinit: [Vertex::default(), Vertex::default()],
        }));

        // Initialize lists
        pinned.efree.init();
        pinned.eused.init();
        pinned.tfree.init();
        pinned.tused.init();
        pinned.lfree.init();
        for i in 0..maxe {
            e.push(EdgeNode::default());
            l.push(VertexToEdgeNode::default());
            l.push(VertexToEdgeNode::default());

            e[i].insert_after(pinned.efree.prev());
            l[i * 2 + 0].insert_after(pinned.lfree.prev());
            l[i * 2 + 1].insert_after(pinned.lfree.prev());
        }
        for i in 0..maxt {
            t.push(TriangleNode::default());
            t[i].insert_after(pinned.tfree.prev());
        }

        // Initialize the initial edge. To do this, find boundbox by the set of points and make an
        // edge on its lower boundary with a margin */
        let mut bbox_min = [0.; 2];
        let mut bbox_max = [0.; 2];
        bbox_min[0] = v[0].pos.x;
        bbox_min[1] = v[0].pos.y;
        bbox_max[0] = v[0].pos.x;
        bbox_max[1] = v[0].pos.y;
        for i in 0..nv {
            if v[i].pos.x < bbox_min[0] {
                bbox_min[0] = v[i].pos.x;
            }
            if v[i].pos.x > bbox_max[0] {
                bbox_max[0] = v[i].pos.x;
            }
            if v[i].pos.y < bbox_min[1] {
                bbox_min[1] = v[i].pos.y;
            }
            if v[i].pos.y > bbox_max[1] {
                bbox_max[1] = v[i].pos.y;
            }
        }
        pinned.vinit[0].pos.x = bbox_min[0] - (bbox_max[0] - bbox_min[0]) * 0.12;
        pinned.vinit[0].pos.y = bbox_min[1] - (bbox_max[1] - bbox_min[1]) * 0.21;
        pinned.vinit[1].pos.x = bbox_max[0] + (bbox_max[0] - bbox_min[0]) * 0.12;
        pinned.vinit[1].pos.y = pinned.vinit[0].pos.y;
        pinned.vinit[0].edges.init();
        pinned.vinit[1].edges.init();

        Self {
            nv,
            v: Pin::new(v),
            _e: Pin::new(e),
            _t: Pin::new(t),
            _l: Pin::new(l),
            s: Pin::new(s),
            pinned,
        }
    }

    fn process(
        &mut self,
        deep: i32,
        optimize: bool,
        constraints: bool,
    ) -> Result<(), TriangulatorError> {
        self.fix_contours_bugs()?;

        let nobjects = self.prepare_triangulation_objects()?;

        for object in 0..nobjects {
            // Triangulation without restriction.
            // Creates a convex triangulation on the entire set of points
            self.sweep_points(object)?;

            // Grid optimization
            if optimize {
                self.optimize_all(deep, object)?;
            }

            // Inserting structural segments
            if constraints {
                self.handle_constraints(object)?;
            }

            // Remove unnecessary triangles
            self.remove_excess_triangles()?;

            // Grid optimization
            if optimize {
                self.optimize_all(deep, object)?;
            }
        }
        Ok(())
    }

    /// Contour error correction function
    ///
    /// Attempts to correct two types of errors: points with duplicate coordinates and outline
    /// twists. In some fonts such effects are observed. In most cases we can speak of design
    /// errors. Such errors are not critical for a typical rasterizer, but triangulation fails on
    /// such contours. For this reason this feature is introduced. It will become more and more
    /// complex in the future.
    fn fix_contours_bugs(&mut self) -> Result<(), TriangulatorError> {
        unsafe {
            // Trying to deal with duplicate points
            let mut need_resorting = false;
            for i in 0..(self.nv - 1) {
                let v1 = self.s[i];
                let v2 = self.s[i + 1];
                let dx = (*v1).pos.x - (*v2).pos.x;
                let dy = (*v1).pos.y - (*v2).pos.y;
                if dx.abs() > EPSILON || dy.abs() > EPSILON {
                    continue;
                }

                // Duplicates have been encountered. Let's try to move the points
                // so as not to create a collision of contours
                let mut v1dir = [Vec2::ZERO; 2]; // Direction from the point towards its neighbours in the contour
                let mut v2dir = [Vec2::ZERO; 2];
                let mut delta = [Vec2::ZERO; 2];

                v1dir[0] = (*(*v1).prev_in_contour).pos - (*v1).pos;
                v1dir[0] = v1dir[0] * 1e-4 * v1dir[0].length();

                v1dir[1] = (*(*v1).next_in_contour).pos - (*v1).pos;
                v1dir[1] = v1dir[1] * 1e-4 * v1dir[1].length();

                v2dir[0] = (*(*v2).prev_in_contour).pos - (*v2).pos;
                v2dir[0] = v2dir[0] * 1e-4 * v2dir[0].length();

                v2dir[1] = (*(*v2).next_in_contour).pos - (*v2).pos;
                v2dir[1] = v2dir[1] * 1e-4 * v2dir[1].length();

                delta[0] = v1dir[0] + v1dir[1];
                delta[1] = v2dir[0] + v2dir[1];

                (*v1).pos = (*v1).pos + delta[0];
                (*v2).pos = (*v2).pos + delta[1];

                need_resorting = true;
            }
            // Sort the vertex array by y-coordinate
            if need_resorting {
                self.s.sort_by(|a, b| {
                    let v1 = (**a).pos;
                    let v2 = (**b).pos;
                    if v1.y == v2.y {
                        if v1.x == v2.x {
                            Ordering::Equal
                        } else if v1.x > v2.x {
                            Ordering::Greater
                        } else {
                            Ordering::Less
                        }
                    } else {
                        if v1.y > v2.y {
                            Ordering::Greater
                        } else {
                            Ordering::Less
                        }
                    }
                });
            }

            // Try to deal with loop kinks like this:
            //          D|                         D
            // A ________|__ B                     |
            //           | /      change it to     / B    by interchanging B and C in the loop
            //           |/               ________/
            //           C                A       C
            for i in 0..self.nv {
                let a = &mut self.v[i] as *mut Vertex;
                let b = (*a).next_in_contour;
                let c = (*b).next_in_contour;
                let d = (*c).next_in_contour;
                if a == b || a == c || a == d {
                    continue;
                }
                let (mut arg1, mut arg2) = (0., 0.);
                if !lines_cross_args(&*a, &*b, &*c, &*d, &mut arg1, &mut arg2) {
                    continue;
                }
                if !(arg1 > 0.0 && arg1 < 1.0 && arg2 > 0.0 && arg2 < 1.0) {
                    continue;
                }
                (*a).next_in_contour = c;
                (*c).next_in_contour = b;
                (*b).next_in_contour = d;
                (*c).prev_in_contour = a;
                (*b).prev_in_contour = c;
                (*d).prev_in_contour = b;
            }

            Ok(())
        }
    }

    /// Prepares independent triangulation objects
    ///
    /// Some symbols contain multiple independent paths. If they are processed by a single
    /// triangulation pass, there is a high chance of error due to duplicate points and
    /// intersections. Therefore, triangulation is performed independently on such contours. This
    /// function assigns each point its ordinal triangulation number depending on the previously set
    /// contour and nested_to fields. The function returns the number of triangulation objects.
    fn prepare_triangulation_objects(&mut self) -> Result<usize, TriangulatorError> {
        let mut res: usize = 0;
        let mut cont2obj: [i16; 256] = [-1; 256];
        for i in 0..self.nv {
            if self.v[i].contour >= 256 {
                return Err(TriangulatorError::Fail);
            }
            if self.v[i].is_hole {
                continue;
            }
            let c = self.v[i].contour;
            if cont2obj[c] == -1 {
                cont2obj[c] = res as i16;
                res += 1;
            }
        }
        for i in 0..self.nv {
            if self.v[i].is_hole {
                if self.v[i]
                    .nested_to
                    .map(|nested_to| nested_to >= 256)
                    .unwrap_or(true)
                {
                    return Err(TriangulatorError::Fail);
                }
                self.v[i].object = cont2obj[self.v[i].nested_to.unwrap()] as usize;
            } else {
                self.v[i].object = cont2obj[self.v[i].contour] as usize;
            }
        }
        return Ok(res);
    }

    /// Algorithm of linear sweeping (no restrictions)
    ///
    /// Modification of the linear point-spacing algorithm. The result is a convex non-optimal
    /// unconstrained triangulation. The essence of the algorithm: go through the triangulation
    /// points previously sorted by coordinate (y); from each point descend a vertical line to the
    /// noticing broken line. This vertical line intersects a certain segment of the broken curve.
    /// Such a segment and the current point form a new triangle, and then it is excluded from the
    /// noticing polyline, and instead of it are inserted 2 formed edges of the triangle. A
    /// modification of the algorithm is that the resulting ledge is connected to the adjacent
    /// segments of the conspicuous polyline subject to bounding conditions. This approach allows
    /// you to get markedly closer to the optimal graph and often prevents the formation of a highly
    /// needle-shaped conspicuous polyline. After trying all points the triangulation is refined to
    /// convex.
    ///
    /// The complexity of the algorithm tends to linear, which is due to the nature of the
    /// sequence of points: when enumerating the points sorted by (y), it is highly probable that
    /// the neighboring points belong to the same glyph contour, i.e. are distant from each other by
    /// (x) insignificantly. In this case, the descending vertical line on a broken curve is very
    /// likely to fall on a newly created segment of this broken curve. So in most cases you don't
    /// have to look in the list of segments of a broken curve for the segment that intersects the
    /// vertical.
    fn sweep_points(&mut self, object: usize) -> Result<(), TriangulatorError> {
        unsafe {
            // STEP 1: Non-convex triangulation

            // Initialization
            let Some(curr) = self.create_edge(&self.pinned.vinit[0] as *const Vertex as *mut Vertex, &self.pinned.vinit[1] as *const Vertex as *mut Vertex) else { return Err(TriangulatorError::Fail) };
            let mut curr = curr.as_ptr();
            let mut convx = EdgeNode::default();
            convx.init();
            convx.reattach(curr);

            // Cycle through all vertices of the selected contour
            for i in 0..self.nv {
                let v = self.s[i];

                if (*v).object != object {
                    continue;
                }

                // Find the edge directly below the current point
                if (*(*curr).v1).pos.x > (*v).pos.x {
                    // Moving to the left along the shell
                    loop {
                        curr = (*curr).prev();
                        let dx1 = (*(*curr).v1).pos.x - (*v).pos.x;
                        let dx2 = (*(*curr).v2).pos.x - (*v).pos.x;
                        if dx1 * dx2 <= 0. && (dx1 != 0. || dx2 != 0.) {
                            break;
                        }
                    }
                } else if (*(*curr).v2).pos.x < (*v).pos.x {
                    // Moving to the right along the shell
                    loop {
                        curr = (*curr).next();
                        let dx1 = (*(*curr).v1).pos.x - (*v).pos.x;
                        let dx2 = (*(*curr).v2).pos.x - (*v).pos.x;
                        if dx1 * dx2 <= 0. && (dx1 != 0. || dx2 != 0.) {
                            break;
                        }
                    }
                }

                if ((*(*curr).v1).pos.x - (*v).pos.x).abs() <= EPSILON {
                    if ((*(*curr).v1).pos.y - (*v).pos.y).abs() <= EPSILON {
                        return Err(TriangulatorError::Fail);
                    }
                }

                if ((*(*curr).v2).pos.x - (*v).pos.x).abs() <= EPSILON {
                    if ((*(*curr).v2).pos.y - (*v).pos.y).abs() <= EPSILON {
                        return Err(TriangulatorError::Fail);
                    }
                }

                // Create two new edges (to the left and right of the current point)
                let Some(l) = self.create_edge((*curr).v1, v) else { return Err(TriangulatorError::Fail) };
                let Some(r) = self.create_edge(v, (*curr).v2) else { return Err(TriangulatorError::Fail) };
                let mut l = l.as_ptr();
                let mut r = r.as_ptr();

                // Replace in the shell the edge found with two created edges. Observe the property
                // of sorting edges in the envelope by x-coordinate.
                (*l).detach();
                (*r).detach();
                (*l).insert_after(curr);
                (*r).insert_after(l);
                self.pinned.eused.reattach(curr);

                // Register a triangle on all three edges
                if self.create_triangle(l, r, curr).is_none() {
                    return Err(TriangulatorError::Fail);
                }

                // If the edge is vertical, make sure it is covered
                if ((*(*l).v1).pos.x - (*(*l).v2).pos.x).abs() <= EPSILON {
                    let Some(ll) = self.make_convex((*l).prev(), l, l).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    l = ll;
                    if l == std::ptr::null_mut() {
                        return Err(TriangulatorError::Fail);
                    }
                }
                if ((*(*r).v1).pos.x - (*(*r).v2).pos.x).abs() <= EPSILON {
                    let Some(rr) = self.make_convex(r, (*r).next(), r).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    r = rr;
                    if r == std::ptr::null_mut() {
                        return Err(TriangulatorError::Fail);
                    }
                }

                while (*l).prev() != &mut convx as *mut EdgeNode {
                    let Some(tmp) = self.make_convex90((*l).prev(), l, l).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    if tmp == l {
                        break;
                    }
                    l = tmp;
                }

                while (*r).next() != &mut convx as *mut EdgeNode {
                    let Some(tmp) = self.make_convex90(r, (*r).next(), r).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    if tmp == r {
                        break;
                    }
                    r = tmp;
                }

                curr = r; // Maybe L, there is no point in guessing the nature of the data

                if l == std::ptr::null_mut() || r == std::ptr::null_mut() {
                    return Err(TriangulatorError::Fail);
                }
            }

            // STEP 2: Finishing triangulation to convex

            let mut done = true;
            while done {
                done = false;
                let mut e1 = convx.next();
                let mut e2 = (*e1).next();
                while e1 != &mut convx as *mut EdgeNode && e2 != &mut convx as *mut EdgeNode {
                    let Some(e1_) = self.make_convex(e1, e2, e2).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    e1 = e1_;
                    if e1 != e2 {
                        done = true;
                    }
                    e2 = (*e1).next();
                }
            }

            while !convx.empty() {
                let e = convx.next();
                (*e).detach();
                self.pinned.eused.attach(e);
            }

            Ok(())
        }
    }

    fn create_edge(&mut self, v1: *mut Vertex, v2: *mut Vertex) -> Option<NonNull<EdgeNode>> {
        unsafe {
            if self.pinned.efree.empty() || self.pinned.lfree.empty() {
                return None;
            }
            let res = self.pinned.efree.first();
            (*res).detach();
            self.pinned.eused.attach(res);
            (*res).v1 = v1;
            (*res).v2 = v2;
            (*res).alt_cc[0] = Circumcircle::default();
            (*res).alt_cc[1] = Circumcircle::default();
            (*res).tr[0] = std::ptr::null_mut();
            (*res).tr[1] = std::ptr::null_mut();
            self.create_v2e_link(v1, res);
            self.create_v2e_link(v2, res);
            Some(NonNull::new(res).unwrap())
        }
    }

    fn create_v2e_link(
        &mut self,
        v: *mut Vertex,
        e: *mut EdgeNode,
    ) -> Option<NonNull<VertexToEdgeNode>> {
        unsafe {
            if self.pinned.lfree.empty() {
                return None;
            }
            let res = self.pinned.lfree.first();
            (*res).detach();
            (*v).edges.attach(res);
            (*res).edge = e;
            Some(NonNull::new(res).unwrap())
        }
    }

    fn create_triangle(
        &mut self,
        e1: *mut EdgeNode,
        e2: *mut EdgeNode,
        e3: *mut EdgeNode,
    ) -> Option<NonNull<TriangleNode>> {
        unsafe {
            if (*e1).tr[1] != std::ptr::null_mut()
                || (*e2).tr[1] != std::ptr::null_mut()
                || (*e3).tr[1] != std::ptr::null_mut()
            {
                return None;
            }
            if self.pinned.tfree.empty() {
                return None;
            }
            let t = self.pinned.tfree.first();
            (*t).detach();
            self.pinned.tused.attach(t);
            (*t).helper = -1;
            (*t).cc = Circumcircle::default();
            (*e1).tr[1] = (*e1).tr[0];
            (*e1).tr[0] = t;
            (*e2).tr[1] = (*e2).tr[0];
            (*e2).tr[0] = t;
            (*e3).tr[1] = (*e3).tr[0];
            (*e3).tr[0] = t;
            (*t).edge[0] = e1;
            (*t).edge[1] = e2;
            (*t).edge[2] = e3;
            Some(NonNull::new(t).unwrap())
        }
    }

    fn make_convex(
        &mut self,
        e1: *mut EdgeNode,
        e2: *mut EdgeNode,
        ret_default: *mut EdgeNode,
    ) -> Option<NonNull<EdgeNode>> {
        unsafe {
            let mut d = [Vec2::ZERO; 2];
            d[0] = (*(*e1).v2).pos - (*(*e1).v1).pos;
            d[1] = (*(*e2).v2).pos - (*(*e2).v1).pos;

            let cross = d[0].cross(d[1]);
            if cross <= 0. {
                return Some(NonNull::new(ret_default).unwrap());
            }

            let Some(n) = self.create_edge((*e1).v1, (*e2).v2) else { return None };
            let n = n.as_ptr();
            (*n).detach();
            (*n).insert_after(e2);
            self.pinned.eused.reattach(e1);
            self.pinned.eused.reattach(e2);
            if self.create_triangle(e1, e2, n).is_none() {
                return None;
            }
            Some(NonNull::new(n).unwrap())
        }
    }

    fn make_convex90(
        &mut self,
        e1: *mut EdgeNode,
        e2: *mut EdgeNode,
        ret_default: *mut EdgeNode,
    ) -> Option<NonNull<EdgeNode>> {
        unsafe {
            let mut d = [Vec2::ZERO; 2];
            d[0] = (*(*e1).v2).pos - (*(*e1).v1).pos;
            d[1] = (*(*e2).v2).pos - (*(*e2).v1).pos;
            let mut l1 = d[0].length();
            let mut l2 = d[1].length();
            l1 = 1.0 / l1;
            l2 = 1.0 / l2;
            let as_ = d[0].cross(d[1]) * l1 * l2;
            let ac = d[0].dot(d[1]) * l1 * l2;
            if as_ < 0. {
                return Some(NonNull::new(ret_default).unwrap());
            }
            if ac > 0. {
                return Some(NonNull::new(ret_default).unwrap());
            }

            let Some(n) = self.create_edge((*e1).v1, (*e2).v2).map(|nn| nn.as_ptr()) else { return None };
            (*n).detach();
            (*n).insert_after(e2);
            self.pinned.eused.reattach(e1);
            self.pinned.eused.reattach(e2);
            if self.create_triangle(e1, e2, n).is_none() {
                return None;
            }
            Some(NonNull::new(n).unwrap())
        }
    }

    /// Attempt to optimize the entire graph
    fn optimize_all(&mut self, deep: i32, object: usize) -> Result<(), TriangulatorError> {
        unsafe {
            let mut e = self.pinned.eused.next();
            while e != &mut self.pinned.eused {
                let opt = e;
                e = (*e).next();
                if (*(*opt).v1).object != object {
                    continue;
                };
                self.optimize(opt, deep)?;
            }
            Ok(())
        }
    }

    fn optimize(&mut self, e: *mut EdgeNode, mut deep: i32) -> Result<(), TriangulatorError> {
        unsafe {
            if deep <= 0 || (*e).tr[1] == std::ptr::null_mut() {
                return Ok(());
            }
            if (*e).is_contour_edge() {
                return Ok(());
            };

            let o0 = (*(*e).tr[0]).opposite_vert(e);
            let o1 = (*(*e).tr[1]).opposite_vert(e);

            // Let's check the quadrilateral for convexity (convexes are already optimal according to Delaunay)
            if !is_convex_quad((*e).v1, o0, (*e).v2, o1) {
                return Ok(());
            }

            let mut done1 = true;
            if (*(*e).tr[0]).cc.radius == 0. {
                done1 &= calc_circumcircle(
                    (*(*e).v1).pos,
                    (*o0).pos,
                    (*(*e).v2).pos,
                    &mut (*(*e).tr[0]).cc,
                )
            };
            if (*(*e).tr[1]).cc.radius == 0. {
                done1 &= calc_circumcircle(
                    (*(*e).v1).pos,
                    (*o1).pos,
                    (*(*e).v2).pos,
                    &mut (*(*e).tr[1]).cc,
                )
            };

            let mut done2 = true;
            if (*e).alt_cc[0].radius == 0. {
                done2 &=
                    calc_circumcircle((*o0).pos, (*(*e).v1).pos, (*o1).pos, &mut (*e).alt_cc[0])
            };
            if (*e).alt_cc[1].radius == 0. {
                done2 &=
                    calc_circumcircle((*o0).pos, (*(*e).v2).pos, (*o1).pos, &mut (*e).alt_cc[1])
            };

            if !done2 {
                return Ok(());
            }

            if done1 && done2 {
                if (*e).alt_cc[0].radius + (*e).alt_cc[1].radius + EPSILON
                    > (*(*e).tr[0]).cc.radius + (*(*e).tr[1]).cc.radius
                {
                    return Ok(());
                }
            }

            self.flip_edge(e)?;

            deep -= 1;
            if deep == 0 {
                return Ok(());
            }
            self.optimize((*(*e).tr[0]).second_edge(e), deep)?;
            self.optimize((*(*e).tr[1]).second_edge(e), deep)?;
            self.optimize((*(*e).tr[0]).third_edge(e), deep)?;
            self.optimize((*(*e).tr[1]).third_edge(e), deep)?;
            Ok(())
        }
    }

    fn flip_edge(&mut self, e: *mut EdgeNode) -> Result<(), TriangulatorError> {
        // Doing an invariant conversion
        //      v1
        //      /|\                      / \
        //   c / | \ a                c /t0 \ a
        //  B / e|  \ A      ->        /_____\ e->v1
        //    \t1|t0/                  \   e /
        //   d \ | / b                d \t1 / b
        //      \|/                      \ /
        //      v2

        unsafe {
            let mut t0 = (*e).tr[0];
            let mut t1 = (*e).tr[1];
            let mut a = (*t0).second_edge(e);
            let mut b = (*t0).third_edge(e);
            let mut c = (*t1).second_edge(e);
            let mut d = (*t1).third_edge(e);
            if !(*a).has_vert((*e).v1) {
                swap(&mut a, &mut b);
            }
            if !(*c).has_vert((*e).v1) {
                swap(&mut c, &mut d);
            }
            let aa = (*a).common_vert(b);
            let bb = (*c).common_vert(d);

            let t0copy = *t0;
            let t1copy = *t1;
            let ecopy = *e;
            self.free_triangle(t0, false);
            self.free_triangle(t1, false);
            self.change_edge(e, aa, bb);
            let Some(t0_) = self.create_triangle(a, c, e).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
            t0 = t0_;
            let Some(t1_) = self.create_triangle(b, d, e).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
            t1 = t1_;

            // Restoring fields after recreating
            (*t0).helper = t0copy.helper;
            (*t1).helper = t1copy.helper;
            (*t0).cc = ecopy.alt_cc[0];
            (*t1).cc = ecopy.alt_cc[1];
            (*e).alt_cc[0] = t1copy.cc; // flip copy!
            (*e).alt_cc[1] = t0copy.cc;

            Ok(())
        }
    }

    fn free_triangle(&mut self, t: *mut TriangleNode, and_bare_edges: bool) {
        unsafe {
            if (*(*t).edge[0]).tr[0] == t {
                (*(*t).edge[0]).tr[0] = (*(*t).edge[0]).tr[1];
            }
            if (*(*t).edge[1]).tr[0] == t {
                (*(*t).edge[1]).tr[0] = (*(*t).edge[1]).tr[1];
            }
            if (*(*t).edge[2]).tr[0] == t {
                (*(*t).edge[2]).tr[0] = (*(*t).edge[2]).tr[1];
            }
            (*(*t).edge[0]).tr[1] = std::ptr::null_mut();
            (*(*t).edge[1]).tr[1] = std::ptr::null_mut();
            (*(*t).edge[2]).tr[1] = std::ptr::null_mut();
            (*(*t).edge[0]).alt_cc[0] = Circumcircle::default();
            (*(*t).edge[0]).alt_cc[1] = Circumcircle::default();
            (*(*t).edge[1]).alt_cc[0] = Circumcircle::default();
            (*(*t).edge[1]).alt_cc[1] = Circumcircle::default();
            (*(*t).edge[2]).alt_cc[0] = Circumcircle::default();
            (*(*t).edge[2]).alt_cc[1] = Circumcircle::default();
            if and_bare_edges && (*(*t).edge[0]).tr[0] == std::ptr::null_mut() {
                self.free_edge((*t).edge[0]);
            }
            if and_bare_edges && (*(*t).edge[1]).tr[0] == std::ptr::null_mut() {
                self.free_edge((*t).edge[1]);
            }
            if and_bare_edges && (*(*t).edge[2]).tr[0] == std::ptr::null_mut() {
                self.free_edge((*t).edge[2]);
            }
            (*t).detach();
            self.pinned.tfree.attach(t);
        }
    }

    fn free_edge(&mut self, e: *mut EdgeNode) -> bool {
        unsafe {
            if (*e).tr[0] != std::ptr::null_mut() {
                return false;
            }
            let mut l = (*(*e).v1).edges.next();
            while l != &mut (*(*e).v1).edges {
                if (*l).edge == e {
                    (*l).detach();
                    self.pinned.lfree.attach(l);
                    break;
                }
                l = (*l).next();
            }
            let mut l = (*(*e).v2).edges.next();
            while l != &mut (*(*e).v2).edges {
                if (*l).edge == e {
                    (*l).detach();
                    self.pinned.lfree.attach(l);
                    break;
                }
                l = (*l).next();
            }
            (*e).detach();
            self.pinned.efree.attach(e);
            return true;
        }
    }

    fn change_edge(&mut self, e: *mut EdgeNode, v1: *mut Vertex, v2: *mut Vertex) {
        unsafe {
            let prev = (*e).prev();
            self.free_edge(e);
            self.create_edge(v1, v2);
            (*e).detach();
            (*e).insert_after(prev);
        }
    }

    /// Function for successive insertion of structural segments
    fn handle_constraints(&mut self, object: usize) -> Result<(), TriangulatorError> {
        for i in 0..self.nv {
            if self.v[i].object != object {
                continue;
            }
            if find_edge(&mut self.v[i], self.v[i].prev_in_contour) != std::ptr::null_mut() {
                continue;
            }
            self.insert_fixed_edge(
                &self.v[i] as *const Vertex as *mut Vertex,
                self.v[i].prev_in_contour,
            )?;
        }
        Ok(())
    }

    /// Function for inserting one structural segment
    ///
    /// This function is called for all structural segments, that should describe closed contours,
    /// but are missing after performing unconstrained triangulation. The function is one of the
    /// most complicated ones. Further work is to optimize it and make it more stable. If the
    /// initial contours contain errors, it is usually, reveal in the process of work of this
    /// function. So it's always it is always relevant to search for its most stable variation.
    fn insert_fixed_edge(
        &mut self,
        v1: *mut Vertex,
        v2: *mut Vertex,
    ) -> Result<(), TriangulatorError> {
        unsafe {
            let mut track = EdgeNode::default();
            track.init();
            self.find_triangles_track(v1, v2, &mut track as *mut EdgeNode)?;

            // Form two contours from the edges that describe the void formed after removing the
            // triangles (on one and on the other side of the inserted edge v1->v2)

            let mut cntr1 = find_edge(v1, (*track.next()).v1);
            let mut cntr2 = find_edge(v1, (*track.next()).v2);
            (*cntr1).detach();
            (*cntr1).init();
            (*cntr2).detach();
            (*cntr2).init();
            let mut e = track.next();
            while e != &mut track {
                let t = (*e).tr[1];
                let e2dn = (*t).second_edge(e);
                let e3rd = (*t).third_edge(e);
                let c2nd = if e2dn == (*e).next() {
                    std::ptr::null_mut()
                } else {
                    if (*cntr1).connected(e2dn) {
                        cntr1
                    } else {
                        cntr2
                    }
                };
                let c3rd = if e3rd == (*e).next() {
                    std::ptr::null_mut()
                } else {
                    if (*cntr1).connected(e3rd) {
                        cntr1
                    } else {
                        cntr2
                    }
                };
                if c2nd != std::ptr::null_mut() {
                    (*c2nd).reattach(e2dn);
                }
                if c3rd != std::ptr::null_mut() {
                    (*c3rd).reattach(e3rd);
                }
                if c2nd == cntr1 || c3rd == cntr1 {
                    cntr1 = (*cntr1).next();
                }
                if c2nd == cntr2 || c3rd == cntr2 {
                    cntr2 = (*cntr2).next();
                }
                e = (*e).next();
            }
            cntr1 = (*cntr1).next();
            cntr2 = (*cntr2).next();

            // Remove interfering triangles
            let mut e = track.next();
            while e != &mut track {
                if (*e).tr[1] != std::ptr::null_mut() {
                    self.free_triangle((*e).tr[1], false);
                }
                if (*e).tr[0] != std::ptr::null_mut() {
                    self.free_triangle((*e).tr[0], false);
                }
                e = (*e).next();
            }

            // Remove any more unnecessary edges of the track
            while !track.empty() {
                if (*track.next()).is_contour_edge() {
                    return Err(TriangulatorError::Fail);
                }
                self.free_edge(track.next());
            }

            // Check the circuits for continuity. This is a temporary measure.
            let mut v = v1;
            let mut e = cntr1;
            loop {
                if !(*e).has_vert(v) {
                    return Err(TriangulatorError::Fail);
                }
                v = (*e).second_vert(v);
                if (*e).next() != cntr1 {
                    e = (*e).next();
                    continue;
                }
                if v != v2 {
                    return Err(TriangulatorError::Fail);
                }
                break;
            }
            v = v1;
            let mut e = cntr2;
            loop {
                if !(*e).has_vert(v) {
                    return Err(TriangulatorError::Fail);
                }
                v = (*e).second_vert(v);
                if (*e).next() != cntr2 {
                    e = (*e).next();
                    continue;
                }
                if v != v2 {
                    return Err(TriangulatorError::Fail);
                }
                break;
            }

            // Create an insertion edge
            let Some(ins) = self.create_edge(v1, v2).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };

            // We triangulate the resulting hole after deletion with the recursive algorithm. In the
            // process, it will return all edges of lists cntr1 and cntr2 back to the mesher (to the
            // list eused)
            self.triangulate_hole(cntr1, ins)?;
            self.triangulate_hole(cntr2, ins)?;

            Ok(())
        }
    }

    /// Auxiliary function to call insert_fixed_edge
    ///
    /// The reference edge at the start of insert_fixed_edge is the inserted structural segment.
    /// Relative to it, the hole is described by two contours (top and bottom). This hole is formed
    /// before the function is called after removing triangles and edges on the insertion path of
    /// the structural segment. Thus, according to A.V. Skvortsov's classification, this function is
    /// a building function in terms of the "insert and build" algorithm.
    ///
    /// The construction algorithm is described as follows. We look for the nearest point to the
    /// reference edge in the contour, form a triangle on the reference edge and that point, and
    /// repeat the algorithm on two new reference edges formed by the triangle. At that, at the
    /// nearest found point we tear the cntr list.
    fn triangulate_hole(
        &mut self,
        cntr: *mut EdgeNode,
        base: *mut EdgeNode,
    ) -> Result<(), TriangulatorError> {
        unsafe {
            if (*cntr).next() == cntr {
                self.pinned.eused.reattach(cntr);
                return Ok(());
            }

            // If the contour has only 2 faces, then form a triangle
            if (*(*cntr).next()).next() == cntr {
                let Some(t) = self.create_triangle(cntr, (*cntr).next(), base).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                // And let's return the mesher's edges from the outline
                self.pinned.eused.reattach((*t).edge[0]);
                self.pinned.eused.reattach((*t).edge[1]);
                return Ok(());
            }

            // Look for the closest point from the contour to the base edge

            let mut closest_edge: *mut EdgeNode = std::ptr::null_mut();
            let mut closest_vert: *mut Vertex = std::ptr::null_mut();
            let mut closest_proj = 0.; // Range is not normalized

            let base_dir = (*(*base).v2).pos - (*(*base).v1).pos; // Directional vector
            let mut orth = Vec2::ZERO; // Orthogonal vector to vector base
            orth[0] = base_dir[1];
            orth[1] = -base_dir[0];

            let mut e = cntr;
            while (*e).next() != cntr {
                let v = (*e).common_vert((*e).next());
                //                 . v
                //                 |
                //                 | proj
                //                 |
                // base.v1 ._______|_______. base.b2

                let vvec = (*v).pos - (*(*base).v1).pos;
                let proj = orth.dot(vvec).abs();
                if closest_edge == std::ptr::null_mut() || proj < closest_proj {
                    closest_proj = proj;
                    closest_edge = e;
                    closest_vert = v;
                }

                e = (*e).next();
            }

            if closest_vert == (*base).v1 || closest_vert == (*base).v2 {
                return Err(TriangulatorError::Fail);
            }

            // Form a triangle on the base edge and the found point
            let mut l = find_edge((*base).v1, closest_vert);
            if l == std::ptr::null_mut() {
                let Some(l_) = self.create_edge((*base).v1, closest_vert).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                l = l_;
            }
            let mut r = find_edge(closest_vert, (*base).v2);
            if r == std::ptr::null_mut() {
                let Some(r_) = self.create_edge(closest_vert, (*base).v2).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                r = r_;
            }
            if l == std::ptr::null_mut() || r == std::ptr::null_mut() {
                return Err(TriangulatorError::Fail);
            }
            let Some(_t) = self.create_triangle(base, l, r).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };

            // Start the function recursively with the base edge L and R
            // First open the contour by forming 2 other contours
            let new_cntr1_beg = cntr;
            let new_cntr1_end = closest_edge;
            let new_cntr2_beg = (*closest_edge).next();
            let new_cntr2_end = (*cntr).prev();
            (*new_cntr1_beg).set_prev(new_cntr1_end);
            (*new_cntr1_end).set_next(new_cntr1_beg);
            (*new_cntr2_beg).set_prev(new_cntr2_end);
            (*new_cntr2_end).set_next(new_cntr2_beg);

            self.triangulate_hole(new_cntr1_beg, l)?;
            self.triangulate_hole(new_cntr2_beg, r)?;

            Ok(())
        }
    }

    fn find_triangles_track(
        &mut self,
        v1: *mut Vertex,
        v2: *mut Vertex,
        root: *mut EdgeNode,
    ) -> Result<(), TriangulatorError> {
        unsafe {
            let mut tcurr: *mut TriangleNode = std::ptr::null_mut();
            let mut ecurr: *mut EdgeNode = std::ptr::null_mut();

            // Find the triangle and its edge opposite v1, which intersects the segment v1-v2
            let mut v2l = (*v1).edges.next();
            while v2l != &mut (*v1).edges && tcurr == std::ptr::null_mut() {
                // Cycle through the triangles on the current edge
                let mut i = 0;
                while i < 2 && tcurr == std::ptr::null_mut() {
                    let t = (*(*v2l).edge).tr[i];
                    if t == std::ptr::null_mut() {
                        i += 1;
                        continue;
                    }
                    let e = (*t).opposite_edge(v1);
                    if !lines_has_common_point((*v1).pos, (*v2).pos, (*(*e).v1).pos, (*(*e).v2).pos)
                    {
                        i += 1;
                        continue;
                    }
                    tcurr = t;
                    ecurr = e;
                    i += 1;
                }
                v2l = (*v2l).next();
            }

            if tcurr == std::ptr::null_mut() {
                return Err(TriangulatorError::Fail);
            }
            (*tcurr).helper = -2;
            loop {
                if (*ecurr).tr[1] == std::ptr::null_mut() {
                    return Err(TriangulatorError::Fail);
                };
                if (*ecurr).tr[1] == tcurr {
                    swap(&mut (*ecurr).tr[0], &mut (*ecurr).tr[1])
                };
                (*ecurr).detach();
                (*root).insert_last(ecurr);
                tcurr = (*ecurr).tr[1];
                if (*tcurr).helper == -2 {
                    return Err(TriangulatorError::Fail);
                }
                (*tcurr).helper = -2;
                if (*tcurr).opposite_vert(ecurr) == v2 {
                    break;
                }
                let e1 = (*tcurr).second_edge(ecurr);
                let e2 = (*tcurr).third_edge(ecurr);
                if lines_has_common_point((*v1).pos, (*v2).pos, (*(*e1).v1).pos, (*(*e1).v2).pos) {
                    ecurr = e1;
                } else if lines_has_common_point(
                    (*v1).pos,
                    (*v2).pos,
                    (*(*e2).v1).pos,
                    (*(*e2).v2).pos,
                ) {
                    ecurr = e2;
                } else {
                    return Err(TriangulatorError::Fail);
                }
            }

            Ok(())
        }
    }

    /// Removes extra triangles from convex triangulation
    ///
    /// The algorithm is simple and is essentially a variant of even-odd. The traversal of all
    /// triangles of convex triangulation begins with the triangle constructed on an auxiliary edge
    /// below the entire graph. This triangle is marked for deletion as well as all its neighbors
    /// if they are not separated from it by a structural segment. Each time the structural segment
    /// is crossed the sign of deletion is inverted. After all triangles are traversed, the
    /// triangles marked for deletion are deleted. The algorithm is shown to be very efficient with
    /// linear complexity.
    fn remove_excess_triangles(&mut self) -> Result<(), TriangulatorError> {
        unsafe {
            // Let's enumerate the triangles, moving from neighbour to neighbour. If the separating
            // edge lies on the contour, then invert the sign when crossing over it. Then remove
            // all negative triangles.

            let mut root = TriangleNode::default(); // The root of the list to be generated
            root.init();

            // Add the first triangle with a negative sign (helper=0)
            let l = self.pinned.vinit[0].edges.next();
            if l == &mut self.pinned.vinit[0].edges {
                return Err(TriangulatorError::Fail);
            }
            if (*(*l).edge).tr[0] == std::ptr::null_mut() {
                return Err(TriangulatorError::Fail);
            }
            (*(*(*l).edge).tr[0]).helper = 0;
            root.reattach((*(*l).edge).tr[0]);

            let mut t = root.next();
            while t != &mut root {
                for i in 0..3 {
                    let neighbor = if (*(*t).edge[i]).tr[0] == t {
                        (*(*t).edge[i]).tr[1]
                    } else {
                        (*(*t).edge[i]).tr[0]
                    };
                    if neighbor == std::ptr::null_mut() {
                        continue;
                    }
                    if (*neighbor).helper >= 0 {
                        continue;
                    };
                    (*neighbor).helper = if (*(*t).edge[i]).is_contour_edge() {
                        (*t).helper ^ 1
                    } else {
                        (*t).helper
                    };
                    (*neighbor).detach();
                    root.insert_last(neighbor);
                }
                let tmp = t;
                t = (*t).next();

                // Delete, or return to the regular list
                if (*tmp).helper == 0 {
                    self.free_triangle(tmp, true);
                } else {
                    self.pinned.tused.reattach(tmp);
                }
            }

            Ok(())
        }
    }

    fn triangles(&self) -> Vec<[Vec2; 3]> {
        let mut vert: Vec<Vec2> = vec![];
        vert.reserve(self.nv);
        let mut triangles = vec![];

        for i in 0..self.nv {
            vert.push(self.v[i].pos);
        }
        let mut t = self.pinned.tused.next();
        while t != &self.pinned.tused as *const TriangleNode as *mut TriangleNode {
            unsafe {
                if (*(*t).edge[1]).is_contour_edge() {
                    swap(&mut (*t).edge[0], &mut (*t).edge[1])
                } else if (*(*t).edge[2]).is_contour_edge() {
                    swap(&mut (*t).edge[0], &mut (*t).edge[2])
                }
                let mut v1 = (*(*t).edge[0]).common_vert((*t).edge[1]);
                let mut v2 = (*(*t).edge[0]).common_vert((*t).edge[2]);
                let v3 = (*(*t).edge[1]).common_vert((*t).edge[2]);
                let d1 = (*v1).pos - (*v2).pos;
                let d2 = (*v1).pos - (*v3).pos;
                if d1.cross(d2) < 0. {
                    swap(&mut v1, &mut v2);
                }
                triangles.push([(*v1).pos, (*v2).pos, (*v3).pos]);
                t = (*t).next();
            }
        }

        triangles
    }
}

fn lines_cross_args(
    a1: &Vertex,
    a2: &Vertex,
    b1: &Vertex,
    b2: &Vertex,
    aarg: &mut f32,
    barg: &mut f32,
) -> bool {
    //       a1
    //      /
    // b1__/_______ b2
    //    /
    //   a2
    //
    // { c = a1 + (a2 - a1) * d1
    // { c = b1 + (b2 - b1) * d2
    //
    // (a2 - a1) * d1 - (b2 - b1) * d2 + a1 - b1 = 0
    //
    //  { (ax2 - ax1) d1  +  (bx1 - bx2) d2  =  bx1 - ax1
    //  { (ay2 - ay1) d1  +  (by1 - by2) d2  =  by1 - ay1
    //
    //  [a1 b1] [d1] = [c1]
    //  [a2 b2] [d2]   [c2]
    //
    //       a1
    //      /
    // b1__/_______ b2
    //    /
    //   a2
    //
    // { c = a1 + (a2 - a1) * d1
    // { c = b1 + (b2 - b1) * d2
    //
    // (a2 - a1) * d1 - (b2 - b1) * d2 + a1 - b1 = 0
    //
    //  { (ax2 - ax1) d1  +  (bx1 - bx2) d2  =  bx1 - ax1
    //  { (ay2 - ay1) d1  +  (by1 - by2) d2  =  by1 - ay1
    //
    //  [a1 b1] [d1] = [c1]
    //  [a2 b2] [d2]   [c2]

    let aa1 = a2.pos.x - a1.pos.x;
    let bb1 = b1.pos.x - b2.pos.x;
    let aa2 = a2.pos.y - a1.pos.y;
    let bb2 = b1.pos.y - b2.pos.y;
    let cc1 = b1.pos.x - a1.pos.x;
    let cc2 = b1.pos.y - a1.pos.y;

    let mut t = aa1 * bb2 - bb1 * aa2;
    if t.abs() < EPSILON {
        return false;
    }
    t = 1.0 / t;
    *aarg = (cc1 * bb2 - bb1 * cc2) * t;
    *barg = (aa1 * cc2 - cc1 * aa2) * t;
    return true;
}

// return 1 if quad is convex
fn is_convex_quad(
    aa: *const Vertex,
    bb: *const Vertex,
    cc: *const Vertex,
    dd: *const Vertex,
) -> bool {
    unsafe {
        let mut v = [Vec2::ZERO; 4];
        let mut z = [0.; 4];
        v[0] = (*bb).pos - (*aa).pos;
        v[1] = (*cc).pos - (*bb).pos;
        v[2] = (*dd).pos - (*cc).pos;
        v[3] = (*aa).pos - (*dd).pos;
        z[0] = v[0].cross(v[1]);
        z[1] = v[1].cross(v[2]);
        z[2] = v[2].cross(v[3]);
        z[3] = v[3].cross(v[0]);
        if z[0] * z[1] <= 0. {
            return false;
        };
        if z[1] * z[2] <= 0. {
            return false;
        };
        if z[2] * z[3] <= 0. {
            return false;
        };
        if z[3] * z[0] <= 0. {
            return false;
        };
        return true;
    }
}

fn calc_circumcircle(aa: Vec2, bb: Vec2, cc: Vec2, mcc: &mut Circumcircle) -> bool {
    //   { (o[0] - A[0])^2 + (o[1] - A[1])^2 = r^2
    //   { (o[0] - B[0])^2 + (o[1] - B[1])^2 = r^2
    //   { (o[0] - C[0])^2 + (o[1] - C[1])^2 = r^2
    //
    // _ { o[0]^2 - 2 o[0] A[0] + A[0]^2   +   o[1]^2 - 2 o[1] A[1] + A[1]^2   =   r^2
    // _ { o[0]^2 - 2 o[0] B[0] + B[0]^2   +   o[1]^2 - 2 o[1] B[1] + B[1]^2   =   r^2
    //   { o[0]^2 - 2 o[0] C[0] + C[0]^2   +   o[1]^2 - 2 o[1] C[1] + C[1]^2   =   r^2
    //
    //   { o[0] 2 (A[0] - B[0]) + o[1] 2 (A[1] - B[1]) = A[0]^2 + A[1]^2 - B[0]^2 - B[1]^2
    //   { o[0] 2 (B[0] - C[0]) + o[1] 2 (B[1] - C[1]) = B[0]^2 + B[1]^2 - C[0]^2 - C[1]^2

    let mut a = [0.; 4];
    let mut b = [0.; 2];
    a[0] = (aa[0] - bb[0]) * 2.0;
    a[1] = (aa[1] - bb[1]) * 2.0;
    a[2] = (bb[0] - cc[0]) * 2.0;
    a[3] = (bb[1] - cc[1]) * 2.0;
    b[0] = aa[0] * aa[0] + aa[1] * aa[1] - bb[0] * bb[0] - bb[1] * bb[1];
    b[1] = bb[0] * bb[0] + bb[1] * bb[1] - cc[0] * cc[0] - cc[1] * cc[1];

    // linsolver of: a * XY = c
    let det = a[0] * a[3] - a[1] * a[2];
    if det.abs() <= EPSILON {
        return false;
    }
    mcc.center[0] = (b[0] * a[3] - a[1] * b[1]) / det;
    mcc.center[1] = (a[0] * b[1] - b[0] * a[2]) / det;

    // calc r
    let dx = aa[0] - mcc.center[0];
    let dy = aa[1] - mcc.center[1];
    mcc.radius = (dx * dx + dy * dy).sqrt();
    return true;
}

fn find_edge(v1: *mut Vertex, v2: *mut Vertex) -> *mut EdgeNode {
    unsafe {
        let mut v2l = (*v1).edges.next();
        while v2l != &mut (*v1).edges {
            if (*(*v2l).edge).has_vert(v1) && (*(*v2l).edge).has_vert(v2) {
                return (*v2l).edge;
            }
            v2l = (*v2l).next()
        }
        return std::ptr::null_mut();
    }
}

/// Checks if two segments have at least one point in common
///
/// Line segments degenerate to a line are allowed.
///
/// The result corresponds to the geometric representation when two objects (segment or point)
/// coincide or have at least one point in common in the drawing at finite scale approximation
/// (defined by the EPSILON constant)
fn lines_has_common_point(l1p1: Vec2, l1p2: Vec2, l2p1: Vec2, l2p2: Vec2) -> bool {
    let mut e1 = l1p2 - l1p1;
    let mut e2 = Vec2::ZERO;
    let mut len = e1.length_squared();

    // l1 - is it a point?
    if len < EPSILON {
        // ---------->x--------->
        //   vec e1      vec e2
        e1 = l1p1 - l2p1;
        e2 = l2p2 - l1p1;
        if e1.cross(e2).abs() > EPSILON {
            return false;
        }
        return e1.dot(e2) >= -EPSILON;
    }

    // Prepare a basis <e1,e2> such that the projection of the vector (l1p2-l1p1) gives one
    len = 1.0 / len;
    e1[0] *= len;
    e1[1] *= len;
    e2[0] = e1[1];
    e2[1] = -e1[0];

    // Let us project l2 into this basis
    let mut l2 = [Vec2::ZERO; 2];
    l2[0] = l2p1 - l1p1;
    l2[1] = l2p2 - l1p1;
    l2[0] = l2[0].proj(e1, e2);
    l2[1] = l2[1].proj(e1, e2);

    // Check if l2 has common points with the unit segment on the OX axis

    // l2 - is it a point?
    e1 = l2[1] - l2[0];
    len = e1.length_squared();
    if len < EPSILON {
        return (l2[0][0] >= -EPSILON)
            && (l2[0][0] <= 1. + EPSILON)
            && ((l2[0][1]).abs() <= EPSILON);
    }

    // l2 - is it a horizontal straight line?
    if (l2[0][1] - l2[1][1]).abs() <= EPSILON {
        if l2[0][1].abs() > EPSILON {
            return false;
        } // Not on the OX axis
        if l2[0][0] >= -EPSILON && l2[0][0] <= 1. + EPSILON {
            return true;
        }
        if l2[1][0] >= -EPSILON && l2[1][0] <= 1. + EPSILON {
            return true;
        }
        if l2[0][0] * l2[1][0] <= EPSILON {
            return true;
        }
        return false;
    }

    // l2 - is it a vertical straight line?
    if (l2[0][0] - l2[1][0]).abs() <= EPSILON {
        return (l2[0][0] >= -EPSILON)
            && (l2[0][0] <= 1.0 + EPSILON)
            && (l2[0][1] * l2[1][1] <= EPSILON);
    }

    // l2 - is a sloping straight line, check it in the range of y
    if l2[0][1] * l2[1][1] > EPSILON {
        return false;
    }

    // Let us check the intersection with OX by solving the equation kx + b = 0
    // We have checked the expressed cases by the previous conditions
    let (k, b);
    cramer2X2!(k, b, l2[0][0], 1.0, l2[0][1], l2[1][0], 1.0, l2[1][1]);
    let x0 = -b / k;

    return x0 > -EPSILON && x0 <= 1.0 + EPSILON;
}

#[derive(Debug, Snafu)]
pub enum TriangulatorError {
    Fail,
    Incomplete,
}

fn compare_contours(o: &Triangulator, i1: usize, i2: usize) -> bool {
    if o.contours[i1].points.len() != o.contours[i2].points.len() {
        return false;
    }
    for i in 0..o.contours[i1].points.len() {
        let dx = o.contours[i1].points[i].x - o.contours[i2].points[i].x;
        let dy = o.contours[i1].points[i].y - o.contours[i2].points[i].y;
        if dx.abs() > EPSILON || dy.abs() > EPSILON {
            return false;
        }
    }
    return true;
}
