use std::{cmp::Ordering, pin::Pin, ptr::NonNull};

use glam::Vec2;
use snafu::Snafu;

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
        points: Vec<Vec2>,
    ) -> Result<(), TriangulatorError> {
        if points.len() < 3 {
            return Err(TriangulatorError::Incomplete);
        }

        self.total_points += points.len();
        self.contours.push(Contour {
            identifier,
            points,
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

    pub fn triangulate(&self) -> Result<Vec<[Vec2; 3]>, TriangulatorError> {
        if !self.contours.is_empty() {
            let mut mesher = Mesher::new(self);
            mesher.process(128, true, true)?;
            Ok(mesher.triangles())
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
                /*
                           u
                     edge /
                         / <-- dx --> .
                        /            point
                       b

                calculation of distance from edge to point:
                y(x) = b.y + (u.y - b.y) / (u.x - b.x) * (x - b.x)    - the edge line equation
                x(y) = (y - b.y) / (u.y - b.y) * (u.x - b.x) + b.x    - inverse equation of line
                dx = point.x - x(point.y)                             - final equation for distance
                */
                let dy = u.y - b.y;
                if dy.abs() > EPSILON
                /* horizontal edges are not handling */
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

macro_rules! maxv_to_maxt {
    ($maxv:expr) => {
        (($maxv - 3) * 2 + 1)
    };
}

macro_rules! maxt_to_maxe {
    ($maxt:expr) => {
        ($maxt * 2 + 1)
    };
}

struct MVS {
    pos: Vec2,
    contour: usize,
    is_hole: bool,
    nested_to: Option<usize>,
    object: usize,
    edges: V2E,
    next_in_contour: *mut MVS,
    prev_in_contour: *mut MVS,
}

impl Default for MVS {
    fn default() -> Self {
        Self {
            pos: Vec2::ZERO,
            contour: 0,
            is_hole: false,
            nested_to: None,
            object: 0,
            edges: V2E::default(),
            next_in_contour: std::ptr::null_mut(),
            prev_in_contour: std::ptr::null_mut(),
        }
    }
}

#[derive(Clone, Copy)]
struct MES {
    next: *mut MES,
    prev: *mut MES,
    v1: *mut MVS,
    v2: *mut MVS,
    tr: [*mut MTS; 2],
    alt_cc: [MCC; 2],
}

impl Default for MES {
    fn default() -> Self {
        Self {
            next: std::ptr::null_mut(),
            prev: std::ptr::null_mut(),
            v1: std::ptr::null_mut(),
            v2: std::ptr::null_mut(),
            tr: [std::ptr::null_mut(); 2],
            alt_cc: [MCC::default(); 2],
        }
    }
}

#[derive(Clone, Copy)]
struct MTS {
    next: *mut MTS,
    prev: *mut MTS,
    edge: [*mut MES; 3],
    cc: MCC,
    helper: i32,
}

impl Default for MTS {
    fn default() -> Self {
        Self {
            next: std::ptr::null_mut(),
            prev: std::ptr::null_mut(),
            edge: [std::ptr::null_mut(); 3],
            cc: MCC::default(),
            helper: 0,
        }
    }
}

struct V2E {
    next: *mut V2E,
    prev: *mut V2E,
    edge: *mut MES,
}

impl Default for V2E {
    fn default() -> Self {
        Self {
            next: std::ptr::null_mut(),
            prev: std::ptr::null_mut(),
            edge: std::ptr::null_mut(),
        }
    }
}

#[derive(Clone, Copy)]
struct MCC {
    radius: f32,
    center: Vec2,
}

impl Default for MCC {
    fn default() -> Self {
        Self {
            radius: 0.,
            center: Vec2::ZERO,
        }
    }
}

macro_rules! list_init {
    ($list:expr) => {
        ($list).next = $list;
        ($list).prev = $list;
    };
}

macro_rules! list_insert_after {
    ($what:expr, $where:expr) => {
        ($what).prev = ($where);
        ($what).next = (*$where).next;
        (*($what).prev).next = ($what);
        (*($what).next).prev = ($what);
    };
}

macro_rules! list_empty {
    ($root:expr) => {
        ($root).next == ($root)
    };
}

macro_rules! list_first {
    ($root:expr) => {
        (*$root).next
    };
}

macro_rules! list_detach {
    ($element:expr) => {
        (*(*$element).prev).next = (*$element).next;
        (*(*$element).next).prev = (*$element).prev;
    };
}

macro_rules! list_attach {
    ($root:expr, $element:expr) => {
        (*$element).prev = ($root);
        (*$element).next = (*$root).next;
        (*(*$root).next).prev = ($element);
        (*$root).next = ($element);
    };
}

macro_rules! list_reattach {
    ($new_root:expr, $element:expr) => {
        list_detach!($element);
        list_attach!($new_root, $element);
    };
}

macro_rules! list_insert_last {
    ($root:expr, $element:expr) => {
        list_insert_after!($element, (*$root).prev)
    };
}

macro_rules! to_mut_ptr {
    ($expr:expr, $type:ty) => {
        ($expr as *const $type as *mut $type)
    };
}

macro_rules! force_borrow {
    ($expr:expr, $type:ty) => {
        &mut *($expr as *const $type as *mut $type)
    };
}

macro_rules! force_pinned_borrow {
    ($expr:expr) => {
        force_borrow!($expr, Pin<Box<MesherPinned>>)
    };
}

macro_rules! vec_cross {
    ($a:expr, $b:expr) => {
        $a.x * $b.y - $a.y * $b.x
    };
}

macro_rules! vec_proj {
    ($res:expr, $vec:expr, $e1:expr, $e2:expr) => {{
        let t = $vec.dot($e1);
        $res[1] = $vec.dot($e2);
        $res[0] = t;
    }};
}

macro_rules! is_contour_edge {
    ($e:expr) => {
        ((*(*$e).v1).prev_in_contour == (*$e).v2 || (*(*$e).v2).prev_in_contour == (*$e).v1)
    };
}

macro_rules! swap {
    ($a:expr, $b:expr) => {{
        let tmp = $a;
        $a = $b;
        $b = tmp;
    }};
}

macro_rules! edges_common_vert {
    ($e1:expr, $e2:expr) => {
        if (*$e1).v1 == (*$e2).v1 || (*$e1).v1 == (*$e2).v2 {
            (*$e1).v1
        } else {
            (*$e1).v2
        }
    };
}

macro_rules! tri_second_edge {
    ($t:expr, $first:expr) => {
        if (*$t).edge[0] == ($first) {
            (*$t).edge[1]
        } else {
            (*$t).edge[0]
        }
    };
}

macro_rules! tri_third_edge {
    ($t:expr, $first:expr) => {
        if (*$t).edge[2] == ($first) {
            (*$t).edge[1]
        } else {
            (*$t).edge[2]
        }
    };
}

macro_rules! opposite_vert {
    ($t:expr, $e:expr) => {
        (edges_common_vert!(tri_second_edge!($t, $e), tri_third_edge!($t, $e)))
    };
}

macro_rules! edge_has_vert {
    ($e:expr, $v:expr) => {
        ((*$e).v1 == ($v) || (*$e).v2 == ($v))
    };
}

macro_rules! edge_second_vert {
    ($e:expr, $v:expr) => {
        if (*$e).v1 == $v {
            (*$e).v2
        } else {
            (*$e).v1
        }
    };
}

macro_rules! edges_connected {
    ($e1:expr, $e2:expr) => {
        edge_has_vert!($e1, (*$e2).v1) || edge_has_vert!($e1, (*$e2).v2)
    };
}

macro_rules! opposite_edge {
    ($t:expr, $v:expr) => {
        if edge_has_vert!((*$t).edge[0], $v) {
            if edge_has_vert!((*$t).edge[1], $v) {
                (*$t).edge[2]
            } else {
                (*$t).edge[1]
            }
        } else {
            (*$t).edge[0]
        }
    };
}

/* Cramer's rule for 2x2 system */
macro_rules! cramer2X2 {
    ($x:expr, $y:expr, $a11:expr, $a12:expr, $c1:expr, $a21:expr, $a22:expr, $c2:expr) => {{
        let det = ($a11) * ($a22) - ($a12) * ($a21);
        $x = (($c1) * ($a22) - ($c2) * ($a12)) / det;
        $y = (($c2) * ($a11) - ($c1) * ($a21)) / det;
    }};
}

struct Mesher {
    nv: usize,               /* Actual number of vertices (may be less than maxv) */
    v: Pin<Vec<MVS>>,        /* Vertex pool, length of maxv */
    _e: Pin<Vec<MES>>,       /* Edge pool, length of maxe */
    _t: Pin<Vec<MTS>>,       /* Triangle pool, length of maxt */
    _l: Pin<Vec<V2E>>,       /* Vertex->edge pool, length of (2 * maxe) */
    s: Pin<Vec<*const MVS>>, /* Sorted by (y) array of vertices */
    pinned: Pin<Box<MesherPinned>>,
}

struct MesherPinned {
    efree: MES,      /* Root of the list of free edges */
    eused: MES,      /* Root of the list of edges used */
    tfree: MTS,      /* Root of the list of free triangles */
    tused: MTS,      /* Root of the list of triangles used */
    lfree: V2E,      /* Root of free reference list vertex->edge */
    convx: MES,      /* Root of the edge list in the convex shell */
    vinit: [MVS; 2], /* Two initialisation points along the lower edge of the glyph */
}

impl Mesher {
    fn new(o: &Triangulator) -> Self {
        // TODO: error on bad contours / no contours

        /* Allocate memory and initialize fields */

        let maxv = o.total_points;
        let maxt = maxv_to_maxt!(maxv + 2); /* Two vertex - initialization */
        let maxe = maxt_to_maxe!(maxt);
        let maxv2e = maxe * 2;

        let mut v: Vec<MVS> = Vec::new();
        v.reserve(maxv);

        let mut e: Vec<MES> = Vec::new();
        e.reserve(maxe);

        let mut t: Vec<MTS> = Vec::new();
        t.reserve(maxt);

        let mut l: Vec<V2E> = Vec::new();
        l.reserve(maxv2e);

        let mut s: Vec<*const MVS> = Vec::new();
        s.reserve(maxv);

        /* Filling in the vertices according to outline points */
        //mvs_t *v = m->v;
        //mvs_t **s = m->s;
        for i in 0..o.contours.len() {
            let pt = &o.contours[i].points;
            let len = pt.len();
            let base = v.len();
            if len < 3 {
                continue;
            }
            /* Byte-by-byte contour matching is done to combat repetition. */
            /* Designers sometimes allow. Example U+2592 in multiple fonts. */
            /* TODO: bool duplicated = false;
            for (int j = 0; j < i && !duplicated; j++)
                duplicated = compare_contours(o, i, j);
            if (duplicated) { continue; } */

            /* Define the type of contour (normal or hole) and if hole, who is the parent */
            let nested_to = o.contours[i].nested_to;
            let is_hole = o.contours[i].is_hole;

            for j in 0..len {
                let vertex = MVS {
                    pos: Vec2::new(pt[j].x, pt[j].y),
                    contour: i,
                    is_hole,
                    nested_to,
                    edges: V2E::default(),
                    // TODO: determine these values here
                    prev_in_contour: std::ptr::null_mut(),
                    next_in_contour: std::ptr::null_mut(),
                    ..Default::default()
                };
                v.push(vertex);
                let v_len = v.len();
                s.push(&v[v_len - 1]);
                list_init!(&mut v[v_len - 1].edges);
            }
            for index in 0..len {
                let next_index = (index + 1) % len;
                let prev_index = (index + len - 1) % len;
                let next_ptr = to_mut_ptr!(&v[base + next_index], MVS);
                let prev_ptr = to_mut_ptr!(&v[base + prev_index], MVS);
                v[base + index].next_in_contour = next_ptr;
                v[base + index].prev_in_contour = prev_ptr;
            }
        }

        let nv = v.len();

        /* Sort the vertex array by y-coordinate */
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
            efree: MES::default(),
            eused: MES::default(),
            tfree: MTS::default(),
            tused: MTS::default(),
            lfree: V2E::default(),
            convx: MES::default(),
            vinit: [MVS::default(), MVS::default()],
        }));

        /* Initialize lists */
        list_init!(&mut pinned.efree);
        list_init!(&mut pinned.eused);
        list_init!(&mut pinned.tfree);
        list_init!(&mut pinned.tused);
        list_init!(&mut pinned.lfree);
        list_init!(&mut pinned.convx);
        for i in 0..maxe {
            e.push(MES::default());
            l.push(V2E::default());
            l.push(V2E::default());
            unsafe {
                list_insert_after!(&mut e[i], pinned.efree.prev);
                list_insert_after!(&mut l[i * 2 + 0], pinned.lfree.prev);
                list_insert_after!(&mut l[i * 2 + 1], pinned.lfree.prev);
            }
        }
        for i in 0..maxt {
            t.push(MTS {
                ..Default::default()
            });
            unsafe {
                list_insert_after!(&mut t[i], pinned.tfree.prev);
            }
        }

        /* Initialize the initial edge. To do this, find */
        /* boundbox by the set of points and make an edge */
        /* on its lower boundary with a margin */
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
        list_init!(&mut pinned.vinit[0].edges);
        list_init!(&mut pinned.vinit[1].edges);

        /*
        /* Инициализируем отладочные поля */
        m->debug.stop_at_step = -1;
        m->debug.curr_step = 0;
        m->debug.message[0] = 0;
        m->debug.breakpoint = false;
        */

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
            /* Triangulation without restriction. */
            /* Creates a convex triangulation on the entire set of points */
            self.sweep_points(object)?;

            /* Grid optimization */
            if optimize {
                self.optimize_all(deep, object)?;
            }

            /* Inserting structural segments */
            if constraints {
                self.handle_constraints(object)?;
            }

            /* Remove unnecessary triangles */
            self.remove_excess_triangles()?;

            /* Grid optimization */
            if optimize {
                self.optimize_all(deep, object)?;
            }
        }
        Ok(())
    }

    /**
     * @brief Contour error correction function
     *
     * Attempts to correct two types of errors: points with duplicate
     * coordinates and outline twists. In some fonts such effects are
     * observed. In most cases we can speak of design errors. Such
     * errors are not critical for a typical rasterizer, but
     * triangulation fails on such contours. For this reason this
     * feature is introduced. It will become more and more complex
     * in the future.
     */
    fn fix_contours_bugs(&mut self) -> Result<(), TriangulatorError> {
        unsafe {
            /* Trying to deal with duplicate points */
            let mut need_resorting = false;
            for i in 0..(self.nv - 1) {
                let v1 = self.s[i] as *mut MVS;
                let v2 = self.s[i + 1] as *mut MVS;
                let dx = (*v1).pos.x - (*v2).pos.x;
                let dy = (*v1).pos.y - (*v2).pos.y;
                if dx.abs() > EPSILON || dy.abs() > EPSILON {
                    continue;
                }

                /* Duplicates have been encountered. Let's try to move the points */
                /* so as not to create a collision of contours */
                let mut v1dir = [Vec2::ZERO; 2]; /* Direction from the point towards its neighbours in the contour */
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
            /* Sort the vertex array by y-coordinate */
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

            /* Trying to deal with loop kinks like this:
                     D|                         D
            A ________|__ B                     |
                      | /      change it to     / B    by interchanging B and C in the loop
                      |/               ________/
                      C                A       C
            */
            for i in 0..self.nv {
                let a = &self.v[i] as *const MVS as *mut MVS;
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
                /*
                arg1 = fabsf(arg1 - 0.5f) * 2;
                arg2 = fabsf(arg2 - 0.5f) * 2;
                if (!(arg1 > 0.8f && arg1 < 1.0f && arg2 > 0.8f && arg2 < 1.0f)) continue;
                */
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

    /**
     * @brief Prepares independent triangulation objects
     *
     * Some symbols contain multiple independent paths. If they are
     * processed by a single triangulation pass, there is a high
     * chance of error due to duplicate points and intersections.
     * Therefore, triangulation is performed independently on such
     * contours. This function assigns each point its ordinal
     * triangulation number depending on the previously set contour
     * and nested_to fields. The function returns the number of
     * triangulation objects.
     */
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

    /**
     * @brief Алгоритм линейного заметания (без ограничений)
     * @param m Структура мешера
     * @return MESHER_XXX
     *
     * Модификация линейного алгоритма заметания точек. Результатом
     * работы является выпуклая неоптимальная триангуляция без
     * ограничений. Суть алгоритма: перебираются точки триангуляции
     * ранее отсортированные по координате (y); из каждой точки
     * опускается вертикаль на заметающую ломаную. Эта вертикаль
     * пересекает определённый отрезок в составе ломаной кривой.
     * Такой отрезок и текущая точка составляют новый треугольник,
     * после чего он исключается из заметающей ломаной, а заместо
     * него вставляются 2 образовавшихся ребра треугольника.
     * Модификация алгоритма состоит в том, что образовавшийся выступ
     * соединяется с соседними отрезками заметающей ломаной при
     * соблюдении ограничивающих условий. Такой подход позволяет
     * заметно приблизиться к оптимальному графу и зачастую
     * предотвращает образование сильно игольчатой заметающей ломаной.
     * После перебора всех точек триангуляция достраивается до
     * выпуклой.
     * Сложность алгоритма стремится к линейной, что обусловлено
     * характером следования точек: при переборе отсортированных
     * по (y) точек велика вероятность, что соседние точки принадлежат
     * одному контуру глифа, то есть удалены друг от друга по (x)
     * незначительно. В этом случае опустившаяся вертикаль на ломаную
     * кривую со значительной вероятностью попадёт на недавно созданный
     * отрезок этой ломаной. Поэтому в большинстве случаев не
     * приходится искать в списке отрезков ломаной тот отрезок, который
     * пересекает вертикаль.
     */
    fn sweep_points(&mut self, object: usize) -> Result<(), TriangulatorError> {
        unsafe {
            /* STEP 1: Non-convex triangulation */

            /* Initialization */
            let Some(curr) = self.create_edge(&self.pinned.vinit[0], &self.pinned.vinit[1]) else { return Err(TriangulatorError::Fail) };
            let mut curr = curr.as_ptr();
            list_init!(&mut force_pinned_borrow!(&self.pinned).convx);
            list_reattach!(&mut force_pinned_borrow!(&self.pinned).convx, curr);

            /* Cycle through all vertices of the selected contour */
            for i in 0..self.nv {
                let v = self.s[i];

                if (*v).object != object {
                    continue;
                }

                /* Find the edge directly below the current point */
                if (*(*curr).v1).pos.x > (*v).pos.x {
                    /* Moving to the left along the shell */
                    loop {
                        curr = (*curr).prev;
                        let dx1 = (*(*curr).v1).pos.x - (*v).pos.x;
                        let dx2 = (*(*curr).v2).pos.x - (*v).pos.x;
                        if dx1 * dx2 <= 0. && (dx1 != 0. || dx2 != 0.) {
                            break;
                        }
                    }
                } else if (*(*curr).v2).pos.x < (*v).pos.x {
                    /* Moving to the right along the shell */
                    loop {
                        curr = (*curr).next;
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

                /* Create two new edges (to the left and right of the current point) */
                let Some(l) = self.create_edge((*curr).v1, v) else { return Err(TriangulatorError::Fail) };
                let Some(r) = self.create_edge(v, (*curr).v2) else { return Err(TriangulatorError::Fail) };
                let mut l = l.as_ptr();
                let mut r = r.as_ptr();

                /* Replace in the shell the edge found with two created edges. Observe
                the property of sorting edges in the envelope by x-coordinate. */
                list_detach!(l);
                list_detach!(r);
                list_insert_after!(&mut *l, curr);
                list_insert_after!(&mut *r, l);
                list_reattach!(&mut force_pinned_borrow!(&self.pinned).eused, curr);

                /* Register a triangle on all three edges */
                if self.create_triangle(l, r, curr).is_none() {
                    return Err(TriangulatorError::Fail);
                }

                /* If the edge is vertical, make sure it is covered */
                if ((*(*l).v1).pos.x - (*(*l).v2).pos.x).abs() <= EPSILON {
                    let Some(ll) = self.make_convex((*l).prev, l, l).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    l = ll;
                    if l == std::ptr::null_mut() {
                        return Err(TriangulatorError::Fail);
                    }
                }
                if ((*(*r).v1).pos.x - (*(*r).v2).pos.x).abs() <= EPSILON {
                    let Some(rr) = self.make_convex(r, (*r).next, r).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    r = rr;
                    if r == std::ptr::null_mut() {
                        return Err(TriangulatorError::Fail);
                    }
                }

                while (*l).prev != &self.pinned.convx as *const MES as *mut MES {
                    let Some(tmp) = self.make_convex90((*l).prev, l, l).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    if tmp == l {
                        break;
                    }
                    l = tmp;
                }

                while (*r).next != &self.pinned.convx as *const MES as *mut MES {
                    let Some(tmp) = self.make_convex90(r, (*r).next, r).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    if tmp == r {
                        break;
                    }
                    r = tmp;
                }

                curr = r; /* Maybe L, there is no point in guessing the nature of the data */

                if l == std::ptr::null_mut() || r == std::ptr::null_mut() {
                    return Err(TriangulatorError::Fail);
                }
            }

            /* ЭТАП 2. Достраивание триангуляции до выпуклой */

            let mut done = true;
            while done {
                done = false;
                let mut e1 = self.pinned.convx.next;
                let mut e2 = (*e1).next;
                while e1 != &self.pinned.convx as *const MES as *mut MES
                    && e2 != &self.pinned.convx as *const MES as *mut MES
                {
                    let Some(e1_) = self.make_convex(e1, e2, e2).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                    e1 = e1_;
                    if e1 != e2 {
                        done = true;
                    }
                    e2 = (*e1).next;
                }
            }

            while !list_empty!(&mut force_pinned_borrow!(&self.pinned).convx) {
                let e = self.pinned.convx.next;
                list_detach!(e);
                list_attach!(&mut force_pinned_borrow!(&self.pinned).eused, e);
            }

            Ok(())
        }
    }

    fn create_edge(&mut self, v1: *const MVS, v2: *const MVS) -> Option<NonNull<MES>> {
        unsafe {
            if list_empty!(&mut force_pinned_borrow!(&self.pinned).efree)
                || list_empty!(&mut force_pinned_borrow!(&self.pinned).lfree)
            {
                return None;
            }
            let res = list_first!(&mut self.pinned.efree);
            list_detach!(res);
            list_attach!(&mut self.pinned.eused, res);
            (*res).v1 = v1 as *mut MVS;
            (*res).v2 = v2 as *mut MVS;
            (*res).alt_cc[0] = MCC::default();
            (*res).alt_cc[1] = MCC::default();
            (*res).tr[0] = std::ptr::null_mut();
            (*res).tr[1] = std::ptr::null_mut();
            self.create_v2e_link(v1, res);
            self.create_v2e_link(v2, res);
            Some(NonNull::new(res).unwrap())
        }
    }

    fn create_v2e_link(&mut self, v: *const MVS, e: *const MES) -> Option<NonNull<V2E>> {
        unsafe {
            if list_empty!(&mut force_pinned_borrow!(&self.pinned).lfree) {
                return None;
            }
            let res = list_first!(&self.pinned.lfree);
            list_detach!(res);
            list_attach!(&mut (*(v as *mut MVS)).edges, res);
            (*res).edge = e as *mut MES;
            Some(NonNull::new(res).unwrap())
        }
    }

    fn create_triangle(
        &mut self,
        e1: *mut MES,
        e2: *mut MES,
        e3: *mut MES,
    ) -> Option<NonNull<MTS>> {
        unsafe {
            if (*e1).tr[1] != std::ptr::null_mut()
                || (*e2).tr[1] != std::ptr::null_mut()
                || (*e3).tr[1] != std::ptr::null_mut()
            {
                return None;
            }
            if list_empty!(&mut force_pinned_borrow!(&self.pinned).tfree) {
                return None;
            }
            let t = list_first!(&force_pinned_borrow!(&self.pinned).tfree);
            list_detach!(t);
            list_attach!(&mut force_pinned_borrow!(&self.pinned).tused, t);
            (*t).helper = -1;
            (*t).cc = MCC::default();
            (*e1).tr[1] = (*e1).tr[0];
            (*e1).tr[0] = t;
            (*e2).tr[1] = (*e2).tr[0];
            (*e2).tr[0] = t;
            (*e3).tr[1] = (*e3).tr[0];
            (*e3).tr[0] = t;
            (*t).edge[0] = e1 as *mut MES;
            (*t).edge[1] = e2 as *mut MES;
            (*t).edge[2] = e3 as *mut MES;
            Some(NonNull::new(t).unwrap())
        }
    }

    fn make_convex(
        &mut self,
        e1: *const MES,
        e2: *const MES,
        ret_default: *const MES,
    ) -> Option<NonNull<MES>> {
        unsafe {
            let mut d = [Vec2::ZERO; 2];
            d[0] = (*(*e1).v2).pos - (*(*e1).v1).pos;
            d[1] = (*(*e2).v2).pos - (*(*e2).v1).pos;

            let cross = vec_cross!(d[0], d[1]);
            if cross <= 0. {
                return Some(NonNull::new(ret_default as *mut MES).unwrap());
            }

            let Some(n) = self.create_edge((*e1).v1, (*e2).v2) else { return None };
            let n = n.as_ptr();
            list_detach!(n);
            list_insert_after!(&mut *n, e2 as *mut MES);
            list_reattach!(
                &mut force_pinned_borrow!(&self.pinned).eused,
                e1 as *mut MES
            );
            list_reattach!(
                &mut force_pinned_borrow!(&self.pinned).eused,
                e2 as *mut MES
            );
            if self
                .create_triangle(e1 as *mut MES, e2 as *mut MES, n)
                .is_none()
            {
                return None;
            }
            Some(NonNull::new(n).unwrap())
        }
    }

    fn make_convex90(
        &mut self,
        e1: *const MES,
        e2: *const MES,
        ret_default: *const MES,
    ) -> Option<NonNull<MES>> {
        unsafe {
            let mut d = [Vec2::ZERO; 2];
            d[0] = (*(*e1).v2).pos - (*(*e1).v1).pos;
            d[1] = (*(*e2).v2).pos - (*(*e2).v1).pos;
            let mut l1 = d[0].length();
            let mut l2 = d[1].length();
            l1 = 1.0 / l1;
            l2 = 1.0 / l2;
            let as_ = vec_cross!(d[0], d[1]) * l1 * l2;
            let ac = d[0].dot(d[1]) * l1 * l2;
            if as_ < 0. {
                return Some(NonNull::new(ret_default as *mut MES).unwrap());
            }
            if ac > 0. {
                return Some(NonNull::new(ret_default as *mut MES).unwrap());
            }

            let Some(n) = self.create_edge((*e1).v1, (*e2).v2).map(|nn| nn.as_ptr()) else { return None };
            list_detach!(n);
            list_insert_after!(&mut *n, e2 as *mut MES);
            list_reattach!(
                &mut force_pinned_borrow!(&self.pinned).eused,
                e1 as *mut MES
            );
            list_reattach!(
                &mut force_pinned_borrow!(&self.pinned).eused,
                e2 as *mut MES
            );
            if self
                .create_triangle(e1 as *mut MES, e2 as *mut MES, n)
                .is_none()
            {
                return None;
            }
            Some(NonNull::new(n).unwrap())
        }
    }

    fn optimize_all(&mut self, deep: i32, object: usize) -> Result<(), TriangulatorError> {
        unsafe {
            let mut e = self.pinned.eused.next;
            while e != &self.pinned.eused as *const MES as *mut MES {
                let opt = e;
                e = (*e).next;
                if (*(*opt).v1).object != object {
                    continue;
                };
                self.optimize(opt, deep)?;
            }
            Ok(())
        }
    }

    fn optimize(&mut self, e: *mut MES, mut deep: i32) -> Result<(), TriangulatorError> {
        unsafe {
            if deep <= 0 || (*e).tr[1] == std::ptr::null_mut() {
                return Ok(());
            }
            if is_contour_edge!(e) {
                return Ok(());
            };

            let o0 = opposite_vert!((*e).tr[0], e);
            let o1 = opposite_vert!((*e).tr[1], e);

            /* Проверим четырёхугольник на выпуклость (впуклые уже оптимальны по Делоне) */
            if !is_convex_quad((*e).v1, o0, (*e).v2, o1) {
                return Ok(());
            }

            /* Проверим условие Делоне */
            //    if (check_delone(e->v1, o0, e->v2, o1)) return MESHER_DONE;

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
            self.optimize(tri_second_edge!((*e).tr[0], e), deep)?;
            self.optimize(tri_second_edge!((*e).tr[1], e), deep)?;
            self.optimize(tri_third_edge!((*e).tr[0], e), deep)?;
            self.optimize(tri_third_edge!((*e).tr[1], e), deep)?;
            Ok(())
        }
    }

    fn flip_edge(&mut self, e: *mut MES) -> Result<(), TriangulatorError> {
        /*
            Делаем инваиантное преобразование

            v1
            /|\                      / \
        c / | \ a                c /t0 \ a
        B / e|  \ A      ->        /_____\ e->v1
            \t1|t0/                  \   e /
        d \ | / b                d \t1 / b
            \|/                      \ /
            v2
            */

        unsafe {
            let mut t0 = (*e).tr[0];
            let mut t1 = (*e).tr[1];
            let mut a = tri_second_edge!(t0, e as *mut MES);
            let mut b = tri_third_edge!(t0, e as *mut MES);
            let mut c = tri_second_edge!(t1, e as *mut MES);
            let mut d = tri_third_edge!(t1, e as *mut MES);
            if !edge_has_vert!(a, (*e).v1) {
                swap!(a, b);
            }
            if !edge_has_vert!(c, (*e).v1) {
                swap!(c, d);
            }
            let aa = edges_common_vert!(a, b);
            let bb = edges_common_vert!(c, d);

            let t0copy = *t0;
            let t1copy = *t1;
            let ecopy = *e;
            self.free_triangle(t0, false);
            self.free_triangle(t1, false);
            self.change_edge(e, aa, bb);
            let Some(t0_) = self.create_triangle(a, c, e as *mut MES).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
            t0 = t0_;
            let Some(t1_) = self.create_triangle(b, d, e as *mut MES).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
            t1 = t1_;

            /* Восстанавливаем поля после пересоздания */
            (*t0).helper = t0copy.helper;
            (*t1).helper = t1copy.helper;
            (*t0).cc = ecopy.alt_cc[0];
            (*t1).cc = ecopy.alt_cc[1];
            (*(e as *mut MES)).alt_cc[0] = t1copy.cc; /* flip copy! */
            (*(e as *mut MES)).alt_cc[1] = t0copy.cc;

            Ok(())
        }
    }

    fn free_triangle(&mut self, t: *const MTS, and_bare_edges: bool) {
        unsafe {
            if (*(*t).edge[0]).tr[0] == t as *mut MTS {
                (*(*t).edge[0]).tr[0] = (*(*t).edge[0]).tr[1];
            }
            if (*(*t).edge[1]).tr[0] == t as *mut MTS {
                (*(*t).edge[1]).tr[0] = (*(*t).edge[1]).tr[1];
            }
            if (*(*t).edge[2]).tr[0] == t as *mut MTS {
                (*(*t).edge[2]).tr[0] = (*(*t).edge[2]).tr[1];
            }
            (*(*t).edge[0]).tr[1] = std::ptr::null_mut();
            (*(*t).edge[1]).tr[1] = std::ptr::null_mut();
            (*(*t).edge[2]).tr[1] = std::ptr::null_mut();
            (*(*t).edge[0]).alt_cc[0] = MCC::default();
            (*(*t).edge[0]).alt_cc[1] = MCC::default();
            (*(*t).edge[1]).alt_cc[0] = MCC::default();
            (*(*t).edge[1]).alt_cc[1] = MCC::default();
            (*(*t).edge[2]).alt_cc[0] = MCC::default();
            (*(*t).edge[2]).alt_cc[1] = MCC::default();
            if and_bare_edges && (*(*t).edge[0]).tr[0] == std::ptr::null_mut() {
                self.free_edge((*t).edge[0]);
            }
            if and_bare_edges && (*(*t).edge[1]).tr[0] == std::ptr::null_mut() {
                self.free_edge((*t).edge[1]);
            }
            if and_bare_edges && (*(*t).edge[2]).tr[0] == std::ptr::null_mut() {
                self.free_edge((*t).edge[2]);
            }
            list_detach!(t);
            list_attach!(&mut force_pinned_borrow!(&self.pinned).tfree, t as *mut MTS);
        }
    }

    fn free_edge(&mut self, e: *const MES) -> bool {
        unsafe {
            if (*e).tr[0] != std::ptr::null_mut() {
                return false;
            }
            let mut l = (*(*e).v1).edges.next;
            while l != &mut (*(*e).v1).edges {
                if (*l).edge == e as *mut MES {
                    list_detach!(l);
                    list_attach!(&mut force_pinned_borrow!(&self.pinned).lfree, l);
                    break;
                }
                l = (*l).next;
            }
            let mut l = (*(*e).v2).edges.next;
            while l != &mut (*(*e).v2).edges {
                if (*l).edge == e as *mut MES {
                    list_detach!(l);
                    list_attach!(&mut force_pinned_borrow!(&self.pinned).lfree, l);
                    break;
                }
                l = (*l).next;
            }
            list_detach!(e);
            list_attach!(&mut force_pinned_borrow!(&self.pinned).efree, e as *mut MES);
            return true;
        }
    }

    fn change_edge(&mut self, e: *const MES, v1: *const MVS, v2: *const MVS) {
        unsafe {
            let prev = (*e).prev;
            self.free_edge(e);
            self.create_edge(v1, v2);
            list_detach!(e);
            list_insert_after!(&mut *(e as *mut MES), prev);
        }
    }

    fn handle_constraints(&mut self, object: usize) -> Result<(), TriangulatorError> {
        for i in 0..self.nv {
            if self.v[i].object != object {
                continue;
            }
            if find_edge(&self.v[i], self.v[i].prev_in_contour) != std::ptr::null() {
                continue;
            }
            self.insert_fixed_edge(&self.v[i], self.v[i].prev_in_contour)?;
        }
        Ok(())
    }

    fn insert_fixed_edge(
        &mut self,
        v1: *const MVS,
        v2: *const MVS,
    ) -> Result<(), TriangulatorError> {
        unsafe {
            let mut track = MES::default();
            list_init!(&mut track);
            self.find_triangles_track(v1 as *mut MVS, v2, &track)?;

            /* Формируем два контура из рёбер, описывающих образовавшуюся после удаления */
            /* треугольников пустоту (с одной и с другой стороны от вставляемого ребра v1->v2) */

            let mut cntr1 = find_edge(v1, (*track.next).v1);
            let mut cntr2 = find_edge(v1, (*track.next).v2);
            list_detach!(cntr1);
            list_init!(&mut *(cntr1 as *mut MES));
            list_detach!(cntr2);
            list_init!(&mut *(cntr2 as *mut MES));
            let mut e = track.next;
            while e != &mut track {
                let t = (*e).tr[1];
                let e2dn = tri_second_edge!(t, e);
                let e3rd = tri_third_edge!(t, e);
                let c2nd = if e2dn == (*e).next {
                    std::ptr::null()
                } else {
                    if edges_connected!(cntr1, e2dn) {
                        cntr1
                    } else {
                        cntr2
                    }
                };
                let c3rd = if e3rd == (*e).next {
                    std::ptr::null()
                } else {
                    if edges_connected!(cntr1, e3rd) {
                        cntr1
                    } else {
                        cntr2
                    }
                };
                if c2nd != std::ptr::null() {
                    list_reattach!(c2nd as *mut MES, e2dn);
                }
                if c3rd != std::ptr::null() {
                    list_reattach!(c3rd as *mut MES, e3rd);
                }
                if c2nd == cntr1 || c3rd == cntr1 {
                    cntr1 = (*cntr1).next;
                }
                if c2nd == cntr2 || c3rd == cntr2 {
                    cntr2 = (*cntr2).next;
                }
                e = (*e).next;
            }
            cntr1 = (*cntr1).next;
            cntr2 = (*cntr2).next;

            /* Удалим мешающие треугольники */
            let mut e = track.next;
            while e != &mut track {
                if (*e).tr[1] != std::ptr::null_mut() {
                    self.free_triangle((*e).tr[1], false);
                }
                if (*e).tr[0] != std::ptr::null_mut() {
                    self.free_triangle((*e).tr[0], false);
                }
                e = (*e).next;
            }

            /* Удаляем больше не нужные грани трека */
            while !list_empty!(force_borrow!(&track, MES)) {
                if is_contour_edge!(track.next) {
                    return Err(TriangulatorError::Fail);
                }
                self.free_edge(track.next);
            }

            /* Проверяем контуры на целостность. Это временная мера. */
            let mut v = v1;
            let mut e = cntr1;
            loop {
                if !edge_has_vert!(e, v as *mut MVS) {
                    return Err(TriangulatorError::Fail);
                }
                v = edge_second_vert!(e, v as *mut MVS);
                if (*e).next != cntr1 as *mut MES {
                    e = (*e).next;
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
                if !edge_has_vert!(e, v as *mut MVS) {
                    return Err(TriangulatorError::Fail);
                }
                v = edge_second_vert!(e, v as *mut MVS);
                if (*e).next != cntr2 as *mut MES {
                    e = (*e).next;
                    continue;
                }
                if v != v2 {
                    return Err(TriangulatorError::Fail);
                }
                break;
            }

            /* Создаём вставляемое ребро */
            let Some(ins) = self.create_edge(v1, v2).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };

            /* Триангулируем получившуюся после удаления дыру рекурсивным */
            /* алгоритмом. В процессе работы он вернёт все грани списков */
            /* cntr1 и cntr2 обратно мешеру (в список eused) */
            self.triangulate_hole(cntr1 as *mut MES, ins)?;
            self.triangulate_hole(cntr2 as *mut MES, ins)?;

            Ok(())
        }
    }

    fn triangulate_hole(
        &mut self,
        cntr: *mut MES,
        base: *mut MES,
    ) -> Result<(), TriangulatorError> {
        unsafe {
            if (*cntr).next == cntr {
                list_reattach!(&mut force_pinned_borrow!(&self.pinned).eused, cntr);
                return Ok(());
            }

            /* Если в контуре только 2 грани, то формируем треугольник */
            if (*(*cntr).next).next == cntr {
                let Some(t) = self.create_triangle(cntr, (*cntr).next, base).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                /* И вернём мешеру грани из контура */
                list_reattach!(&mut force_pinned_borrow!(&self.pinned).eused, (*t).edge[0]);
                list_reattach!(&mut force_pinned_borrow!(&self.pinned).eused, (*t).edge[1]);
                return Ok(());
            }

            /* Ищем самую близкую точку из контура к ребру base */

            let mut closest_edge: *mut MES = std::ptr::null_mut();
            let mut closest_vert: *mut MVS = std::ptr::null_mut();
            let mut closest_proj = 0.; /* Дальность не нормированная */

            let base_dir = (*(*base).v2).pos - (*(*base).v1).pos;
            let mut orth = Vec2::ZERO; /* Ортогональный вектор к вектору base */
            orth[0] = base_dir[1];
            orth[1] = -base_dir[0];

            let mut e = cntr;
            while (*e).next != cntr {
                let v = edges_common_vert!(e, (*e).next);
                /*
                                . v
                                |
                                | proj
                                |
                base.v1 ._______|_______. base.b2
                */

                let vvec = (*v).pos - (*(*base).v1).pos;
                let proj = orth.dot(vvec).abs();
                if closest_edge == std::ptr::null_mut() || proj < closest_proj {
                    closest_proj = proj;
                    closest_edge = e;
                    closest_vert = v;
                }

                e = (*e).next;
            }

            if closest_vert == (*base).v1 || closest_vert == (*base).v2 {
                return Err(TriangulatorError::Fail);
            }

            /* Формируем треугольник на базовом ребре и найденной точке */
            let mut l = find_edge((*base).v1, closest_vert);
            if l == std::ptr::null() {
                let Some(l_) = self.create_edge((*base).v1, closest_vert).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                l = l_;
            }
            let mut r = find_edge(closest_vert, (*base).v2);
            if r == std::ptr::null() {
                let Some(r_) = self.create_edge(closest_vert, (*base).v2).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };
                r = r_;
            }
            if l == std::ptr::null() || r == std::ptr::null() {
                return Err(TriangulatorError::Fail);
            }
            let Some(_t) = self.create_triangle(base, l as *mut MES, r as *mut MES).map(|nn| nn.as_ptr()) else { return Err(TriangulatorError::Fail) };

            /* Запускаем функцию рекурсивно с базовым ребром L и R */
            /* Сначала размыкаем контур, формируя из него 2 других */
            let new_cntr1_beg = cntr;
            let new_cntr1_end = closest_edge;
            let new_cntr2_beg = (*closest_edge).next;
            let new_cntr2_end = (*cntr).prev;
            (*new_cntr1_beg).prev = new_cntr1_end;
            (*new_cntr1_end).next = new_cntr1_beg;
            (*new_cntr2_beg).prev = new_cntr2_end;
            (*new_cntr2_end).next = new_cntr2_beg;

            self.triangulate_hole(new_cntr1_beg, l as *mut MES)?;
            self.triangulate_hole(new_cntr2_beg, r as *mut MES)?;

            Ok(())
        }
    }

    fn find_triangles_track(
        &mut self,
        v1: *mut MVS,
        v2: *const MVS,
        root: *const MES,
    ) -> Result<(), TriangulatorError> {
        unsafe {
            let mut tcurr: *mut MTS = std::ptr::null_mut();
            let mut ecurr: *mut MES = std::ptr::null_mut();

            /* Find the triangle and its edge opposite v1, which intersects the segment v1-v2 */
            let mut v2l = (*v1).edges.next;
            while v2l != &mut (*v1).edges && tcurr == std::ptr::null_mut() {
                /* Cycle through the triangles on the current edge */
                let mut i = 0;
                while i < 2 && tcurr == std::ptr::null_mut() {
                    let t = (*(*v2l).edge).tr[i];
                    if t == std::ptr::null_mut() {
                        i += 1;
                        continue;
                    }
                    let e = opposite_edge!(t, v1);
                    if !lines_has_common_point((*v1).pos, (*v2).pos, (*(*e).v1).pos, (*(*e).v2).pos)
                    {
                        i += 1;
                        continue;
                    }
                    tcurr = t;
                    ecurr = e;
                    i += 1;
                }
                v2l = (*v2l).next;
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
                    swap!((*ecurr).tr[0], (*ecurr).tr[1])
                };
                list_detach!(ecurr);
                list_insert_last!(root, &mut *ecurr);
                tcurr = (*ecurr).tr[1];
                if (*tcurr).helper == -2 {
                    return Err(TriangulatorError::Fail);
                }
                (*tcurr).helper = -2;
                if opposite_vert!(tcurr, ecurr) == v2 as *mut MVS {
                    break;
                }
                let e1 = tri_second_edge!(tcurr, ecurr);
                let e2 = tri_third_edge!(tcurr, ecurr);
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

    fn remove_excess_triangles(&mut self) -> Result<(), TriangulatorError> {
        unsafe {
            /*  Let's enumerate the triangles, moving from neighbour to neighbour */
            /* If the separating edge lies on the contour, then invert the sign when */
            /* crossing over it. Then remove all negative triangles. */

            let mut root = MTS::default(); /* The root of the list to be generated */
            list_init!(&mut root);

            /* Add the first triangle with a negative sign (helper=0) */
            let l = self.pinned.vinit[0].edges.next;
            if l == &mut self.pinned.vinit[0].edges {
                return Err(TriangulatorError::Fail);
            }
            if (*(*l).edge).tr[0] == std::ptr::null_mut() {
                return Err(TriangulatorError::Fail);
            }
            (*(*(*l).edge).tr[0]).helper = 0;
            list_reattach!(&mut root, (*(*l).edge).tr[0]);

            let mut t = root.next;
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
                    (*neighbor).helper = if is_contour_edge!((*t).edge[i]) {
                        (*t).helper ^ 1
                    } else {
                        (*t).helper
                    };
                    list_detach!(neighbor);
                    list_insert_last!(&mut root, &mut *neighbor);
                }
                let mut tmp = t;
                t = (*t).next;

                /* Удаляем, либо возвращаем в штатный список */
                if (*tmp).helper == 0 {
                    self.free_triangle(tmp, true);
                } else {
                    list_reattach!(&mut force_pinned_borrow!(&self.pinned).tused, tmp);
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
        let mut t = self.pinned.tused.next;
        while t != &self.pinned.tused as *const MTS as *mut MTS {
            unsafe {
                //mvs_t *v1, *v2, *v3;
                //float d1[2], d2[2];
                if is_contour_edge!((*t).edge[1]) {
                    swap!((*t).edge[0], (*t).edge[1])
                } else if is_contour_edge!((*t).edge[2]) {
                    swap!((*t).edge[0], (*t).edge[2])
                }
                let mut v1 = edges_common_vert!((*t).edge[0], (*t).edge[1]);
                let mut v2 = edges_common_vert!((*t).edge[0], (*t).edge[2]);
                let v3 = edges_common_vert!((*t).edge[1], (*t).edge[2]);
                let d1 = (*v1).pos - (*v2).pos;
                let d2 = (*v1).pos - (*v3).pos;
                if vec_cross!(d1, d2) < 0. {
                    swap!(v1, v2);
                }
                /*out->faces[out->nfaces].v1 = v1 - mesh->v;
                out->faces[out->nfaces].v2 = v2 - mesh->v;
                out->faces[out->nfaces].v3 = v3 - mesh->v;
                out->nfaces++;*/
                triangles.push([(*v1).pos, (*v2).pos, (*v3).pos]);
                t = (*t).next;
            }
        }

        triangles
    }
}

fn lines_cross_args(
    a1: &MVS,
    a2: &MVS,
    b1: &MVS,
    b2: &MVS,
    aarg: &mut f32,
    barg: &mut f32,
) -> bool {
    /*
          a1
         /
    b1__/_______ b2
       /
      a2
    */

    /* { c = a1 + (a2 - a1) * d1 */
    /* { c = b1 + (b2 - b1) * d2 */

    /* (a2 - a1) * d1 - (b2 - b1) * d2 + a1 - b1 = 0 */

    /*
        { (ax2 - ax1) d1  +  (bx1 - bx2) d2  =  bx1 - ax1
        { (ay2 - ay1) d1  +  (by1 - by2) d2  =  by1 - ay1

        [a1 b1] [d1] = [c1]
        [a2 b2] [d2]   [c2]
    */

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

fn is_convex_quad(aa: *const MVS, bb: *const MVS, cc: *const MVS, dd: *const MVS) -> bool {
    unsafe {
        let mut v = [Vec2::ZERO; 4];
        let mut z = [0.; 4];
        v[0] = (*bb).pos - (*aa).pos;
        v[1] = (*cc).pos - (*bb).pos;
        v[2] = (*dd).pos - (*cc).pos;
        v[3] = (*aa).pos - (*dd).pos;
        z[0] = vec_cross!(v[0], v[1]);
        z[1] = vec_cross!(v[1], v[2]);
        z[2] = vec_cross!(v[2], v[3]);
        z[3] = vec_cross!(v[3], v[0]);
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

fn calc_circumcircle(aa: Vec2, bb: Vec2, cc: Vec2, mcc: &mut MCC) -> bool {
    /*
      { (o[0] - A[0])^2 + (o[1] - A[1])^2 = r^2
      { (o[0] - B[0])^2 + (o[1] - B[1])^2 = r^2
      { (o[0] - C[0])^2 + (o[1] - C[1])^2 = r^2

    _ { o[0]^2 - 2 o[0] A[0] + A[0]^2   +   o[1]^2 - 2 o[1] A[1] + A[1]^2   =   r^2
    _ { o[0]^2 - 2 o[0] B[0] + B[0]^2   +   o[1]^2 - 2 o[1] B[1] + B[1]^2   =   r^2
      { o[0]^2 - 2 o[0] C[0] + C[0]^2   +   o[1]^2 - 2 o[1] C[1] + C[1]^2   =   r^2

      { o[0] 2 (A[0] - B[0]) + o[1] 2 (A[1] - B[1]) = A[0]^2 + A[1]^2 - B[0]^2 - B[1]^2
      { o[0] 2 (B[0] - C[0]) + o[1] 2 (B[1] - C[1]) = B[0]^2 + B[1]^2 - C[0]^2 - C[1]^2
      */

    let mut a = [0.; 4];
    let mut b = [0.; 2];
    a[0] = (aa[0] - bb[0]) * 2.0;
    a[1] = (aa[1] - bb[1]) * 2.0;
    a[2] = (bb[0] - cc[0]) * 2.0;
    a[3] = (bb[1] - cc[1]) * 2.0;
    b[0] = aa[0] * aa[0] + aa[1] * aa[1] - bb[0] * bb[0] - bb[1] * bb[1];
    b[1] = bb[0] * bb[0] + bb[1] * bb[1] - cc[0] * cc[0] - cc[1] * cc[1];

    /* linsolver of: a * XY = c */
    let det = a[0] * a[3] - a[1] * a[2];
    if det.abs() <= EPSILON {
        return false;
    }
    mcc.center[0] = (b[0] * a[3] - a[1] * b[1]) / det;
    mcc.center[1] = (a[0] * b[1] - b[0] * a[2]) / det;

    /* calc r */
    let dx = aa[0] - mcc.center[0];
    let dy = aa[1] - mcc.center[1];
    mcc.radius = (dx * dx + dy * dy).sqrt();
    return true;
}

fn find_edge(v1: *const MVS, v2: *const MVS) -> *const MES {
    unsafe {
        let mut v2l = (*v1).edges.next;
        while v2l != &mut (*(v1 as *mut MVS)).edges {
            if edge_has_vert!((*v2l).edge, v1 as *mut MVS)
                && edge_has_vert!((*v2l).edge, v2 as *mut MVS)
            {
                return (*v2l).edge;
            }
            v2l = (*v2l).next
        }
        return std::ptr::null();
    }
}

fn lines_has_common_point(l1p1: Vec2, l1p2: Vec2, l2p1: Vec2, l2p2: Vec2) -> bool {
    //float e1[2], e2[2], l2[2][2];
    //float len, k, b, x0;

    let mut e1 = l1p2 - l1p1;
    let mut e2 = Vec2::ZERO;
    let mut len = e1.length_squared();

    /* l1 - это точка? */
    if len < EPSILON {
        /* ---------->x---------> */
        /*   vec e1      vec e2   */
        e1 = l1p1 - l2p1;
        e2 = l2p2 - l1p1;
        if vec_cross!(e1, e2).abs() > EPSILON {
            return false;
        }
        return e1.dot(e2) >= -EPSILON;
    }

    /* Подготовим базис <e1,e2> такой что проекция */
    /* вектора (l1p2-l1p1) будет давать единицу */
    len = 1.0 / len;
    e1[0] *= len;
    e1[1] *= len;
    e2[0] = e1[1];
    e2[1] = -e1[0];

    /* Спроецируем l2 в этот базис */
    let mut l2 = [Vec2::ZERO; 2];
    l2[0] = l2p1 - l1p1;
    l2[1] = l2p2 - l1p1;
    vec_proj!(l2[0], l2[0], e1, e2);
    vec_proj!(l2[1], l2[1], e1, e2);

    /* Проверим, имеет ли l2 общие точки c */
    /* единичным отрезком на оси OX */

    /* l2 - это точка? */
    e1 = l2[1] - l2[0];
    len = e1.length_squared();
    if len < EPSILON {
        return (l2[0][0] >= -EPSILON)
            && (l2[0][0] <= 1. + EPSILON)
            && ((l2[0][1]).abs() <= EPSILON);
    }

    /* l2 - это горизонтальная прямая? */
    if (l2[0][1] - l2[1][1]).abs() <= EPSILON {
        if l2[0][1].abs() > EPSILON {
            return false;
        } /* Не на оси OX */
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

    /* l2 - это вертикальная прямая? */
    if (l2[0][0] - l2[1][0]).abs() <= EPSILON {
        return (l2[0][0] >= -EPSILON)
            && (l2[0][0] <= 1.0 + EPSILON)
            && (l2[0][1] * l2[1][1] <= EPSILON);
    }

    /* l2 - это наклонная прямая, проверим её в диапазоне y */
    if l2[0][1] * l2[1][1] > EPSILON {
        return false;
    }

    /* Проверим пересечение с OX решив уравнение kx + b = 0  */
    /* Выражденные случаи мы проверили предыдущими условиями */
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
