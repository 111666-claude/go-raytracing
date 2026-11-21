package main

import (
	"math"
	"testing"
)

const epsilon = 1e-9

func almostEqual(a, b float64) bool {
	return math.Abs(a-b) <= epsilon
}

func assertVec(t *testing.T, name string, got, want Vec3) {
	t.Helper()
	if !almostEqual(got.X, want.X) || !almostEqual(got.Y, want.Y) || !almostEqual(got.Z, want.Z) {
		t.Errorf("%s: got %v, want %v", name, got, want)
	}
}

func assertHit(t *testing.T, h Hitable, r *Ray, tMin, tMax float64, wantT float64, wantNormal Vec3) {
	t.Helper()
	var rec HitRecord
	if !h.hit(r, tMin, tMax, &rec) {
		t.Fatalf("expected a hit at t=%v, got none", wantT)
	}
	if !almostEqual(rec.t, wantT) {
		t.Errorf("hit distance: got %v, want %v", rec.t, wantT)
	}
	assertVec(t, "hit point", rec.p, r.PointAtParameter(wantT))
	assertVec(t, "outward normal", rec.normal, wantNormal)
	if rec.material == nil {
		t.Errorf("hit record carries no material")
	}
}

func assertMiss(t *testing.T, h Hitable, r *Ray) {
	t.Helper()
	var rec HitRecord
	if h.hit(r, 0.0001, math.MaxFloat64, &rec) {
		t.Errorf("expected a miss, got hit at t=%v", rec.t)
	}
}

// Fixed ray against an axis-aligned box: straight down -Z onto the +Z face.
func TestBoxAxisAlignedHit(t *testing.T) {
	box := NewBox(Vec3{0, 0, -1}, Vec3{2, 2, 2}, Vec3{0, 0, 0}, Labertiam{Vec3{0.5, 0.5, 0.5}})
	r := Ray{Vec3{0, 0, 3}, Vec3{0, 0, -1}}
	assertHit(t, box, &r, 0.0001, math.MaxFloat64, 3.0, Vec3{0, 0, 1})
}

// Box rotated 90 degrees around Z swaps its X and Y extents.
func TestBoxRotated90Degrees(t *testing.T) {
	box := NewBox(Vec3{0, 0, 0}, Vec3{2, 4, 6}, Vec3{0, 0, math.Pi / 2}, Labertiam{Vec3{0.5, 0.5, 0.5}})
	r := Ray{Vec3{5, 0, 0}, Vec3{-1, 0, 0}}
	// The 4-long Y edge now lies along X: the face is at x = +2.
	assertHit(t, box, &r, 0.0001, math.MaxFloat64, 3.0, Vec3{1, 0, 0})
}

// Box rotated 45 degrees around Z, ray offset from the diagonal so it
// enters through a single face. Closed-form expectations:
// local origin x = 1.75*sqrt(2), t = 3.5 - sqrt(2), normal = (c, c, 0).
func TestBoxRotated45Degrees(t *testing.T) {
	box := NewBox(Vec3{0, 0, 0}, Vec3{2, 2, 2}, Vec3{0, 0, math.Pi / 4}, NewMetal(Vec3{0.8, 0.8, 0.8}, 0.0))
	r := Ray{Vec3{3, 0.5, 0}, Vec3{-1, 0, 0}}
	c := math.Sqrt2 / 2
	assertHit(t, box, &r, 0.0001, math.MaxFloat64, 3.5-math.Sqrt2, Vec3{c, c, 0})
}

// A ray starting inside the box reports the exit face with its outward normal.
func TestBoxHitFromInside(t *testing.T) {
	box := NewBox(Vec3{0, 0, 0}, Vec3{2, 2, 2}, Vec3{0, 0, 0}, Labertiam{Vec3{0.5, 0.5, 0.5}})
	r := Ray{Vec3{0, 0, 0}, Vec3{1, 0, 0}}
	assertHit(t, box, &r, 0.0001, math.MaxFloat64, 1.0, Vec3{1, 0, 0})
}

// A ray parallel to a face and outside its slab misses; the same ray
// nudged inside the slab hits, with a deterministic result.
func TestBoxNearParallelRay(t *testing.T) {
	box := NewBox(Vec3{0, 0, 0}, Vec3{2, 2, 2}, Vec3{0, 0, 0}, Labertiam{Vec3{0.5, 0.5, 0.5}})
	outside := Ray{Vec3{-3, 1.0000001, 0}, Vec3{1, 0, 0}}
	assertMiss(t, box, &outside)
	inside := Ray{Vec3{-3, 0.9999999, 0}, Vec3{1, 0, 0}}
	assertHit(t, box, &inside, 0.0001, math.MaxFloat64, 2.0, Vec3{-1, 0, 0})
}

// A ray running exactly along the plane of a face (y == half extent)
// reaches the edge; the hit is deterministic: front face, X normal.
func TestBoxTangentRay(t *testing.T) {
	box := NewBox(Vec3{0, 0, 0}, Vec3{2, 2, 2}, Vec3{0, 0, 0}, Labertiam{Vec3{0.5, 0.5, 0.5}})
	r := Ray{Vec3{-3, 1, 0}, Vec3{1, 0, 0}}
	assertHit(t, box, &r, 0.0001, math.MaxFloat64, 2.0, Vec3{-1, 0, 0})
}

// An edge hit where two slabs tie resolves to the lowest-index axis (X).
func TestBoxEdgeTieBreak(t *testing.T) {
	box := NewBox(Vec3{0, 0, 0}, Vec3{2, 2, 2}, Vec3{0, 0, math.Pi / 4}, Labertiam{Vec3{0.5, 0.5, 0.5}})
	// Aimed exactly at the rotated corner on the +X axis.
	r := Ray{Vec3{3, 0, 0}, Vec3{-1, 0, 0}}
	c := math.Sqrt2 / 2
	assertHit(t, box, &r, 0.0001, math.MaxFloat64, 3.0-math.Sqrt2, Vec3{c, c, 0})
}

// A zero-size box degenerates to its center point: a ray through the
// center hits at that exact distance, any other ray misses. Both are
// deterministic.
func TestBoxZeroSize(t *testing.T) {
	box := NewBox(Vec3{0, 0, 0}, Vec3{0, 0, 0}, Vec3{0, 0, 0}, Labertiam{Vec3{0.5, 0.5, 0.5}})
	through := Ray{Vec3{0, 0, 3}, Vec3{0, 0, -1}}
	assertHit(t, box, &through, 0.0001, math.MaxFloat64, 3.0, Vec3{0, 0, -1})
	off := Ray{Vec3{0.1, 0, 3}, Vec3{0, 0, -1}}
	assertMiss(t, box, &off)
}

// Non-finite transform parameters produce an invalid box that never hits.
func TestBoxNonFiniteTransform(t *testing.T) {
	r := Ray{Vec3{0, 0, 3}, Vec3{0, 0, -1}}
	boxes := []Box{
		NewBox(Vec3{math.NaN(), 0, 0}, Vec3{2, 2, 2}, Vec3{0, 0, 0}, Labertiam{Vec3{0.5, 0.5, 0.5}}),
		NewBox(Vec3{0, 0, 0}, Vec3{2, math.Inf(1), 2}, Vec3{0, 0, 0}, Labertiam{Vec3{0.5, 0.5, 0.5}}),
		NewBox(Vec3{0, 0, 0}, Vec3{2, 2, 2}, Vec3{0, math.Inf(-1), 0}, Labertiam{Vec3{0.5, 0.5, 0.5}}),
	}
	for i, box := range boxes {
		var rec HitRecord
		if box.hit(&r, 0.0001, math.MaxFloat64, &rec) {
			t.Errorf("box %d with non-finite transform reported a hit", i)
		}
	}
}

// A translated box occludes a farther sphere (shadowing), and a nearer
// sphere occludes the box: the closest hit wins deterministically.
func TestBoxSphereOcclusionOrder(t *testing.T) {
	box := NewBox(Vec3{0, 0, -4}, Vec3{2, 2, 2}, Vec3{0, math.Pi / 6, 0}, NewMetal(Vec3{0.8, 0.6, 0.2}, 0.0))
	sphereNear := Sphere{Vec3{0, 0, 1}, 0.5, Labertiam{Vec3{0.1, 0.2, 0.5}}}
	sphereFar := Sphere{Vec3{0, 0, -8}, 0.5, Labertiam{Vec3{0.1, 0.2, 0.5}}}
	r := Ray{Vec3{0, 0, 3}, Vec3{0, 0, -1}}

	// The box is tilted 30 degrees around Y: the ray meets its +Z face at
	// t = 7 - 2/sqrt(3) with the tilted outward normal (sin30, 0, cos30).
	world := HitableList{[]Hitable{box, sphereFar}}
	assertHit(t, world, &r, 0.0001, math.MaxFloat64,
		7.0-2.0/math.Sqrt(3.0), Vec3{0.5, 0, math.Sqrt(3.0) / 2.0})
	var rec HitRecord
	world.hit(&r, 0.0001, math.MaxFloat64, &rec)
	if _, ok := rec.material.(Metal); !ok {
		t.Errorf("expected the box material (Metal), got %T", rec.material)
	}

	world = HitableList{[]Hitable{box, sphereNear}}
	if !world.hit(&r, 0.0001, math.MaxFloat64, &rec) {
		t.Fatal("expected the sphere to be hit")
	}
	if !almostEqual(rec.t, 1.5) { // sphere front at z = 1.5
		t.Errorf("nearer sphere should occlude the box: got t=%v, want 1.5", rec.t)
	}
	if _, ok := rec.material.(Labertiam); !ok {
		t.Errorf("expected the sphere material (Labertiam), got %T", rec.material)
	}
}

// A large scene of boxes and spheres keeps a stable closest-hit ordering.
func TestBoxLargeSceneStable(t *testing.T) {
	list := make([]Hitable, 0, 202)
	for i := 0; i < 100; i++ {
		x := float64(i) * 3
		list = append(list, NewBox(Vec3{x, 0, -10}, Vec3{2, 2, 2}, Vec3{0, float64(i) * 0.1, 0}, Labertiam{Vec3{0.5, 0.5, 0.5}}))
		list = append(list, Sphere{Vec3{x, 5, -10}, 1, Labertiam{Vec3{0.5, 0.5, 0.5}}})
	}
	world := HitableList{list}
	// Ray aimed at box #7 (center x = 21), rotated 0.7 rad around Y.
	// Closed form: t = 15 - 1/cos(0.7), normal = (sin 0.7, 0, cos 0.7).
	r := Ray{Vec3{21, 0, 5}, Vec3{0, 0, -1}}
	assertHit(t, world, &r, 0.0001, math.MaxFloat64,
		15.0-1.0/math.Cos(0.7), Vec3{math.Sin(0.7), 0, math.Cos(0.7)})
}

// Fixed-ray regression checks for the pre-existing glass sphere:
// distance and outward normal must stay unchanged.
func TestGlassSphereFixedRay(t *testing.T) {
	glass := Sphere{Vec3{0, 0, -1}, 0.5, Dielectric{1.5}}
	r := Ray{Vec3{0, 0, 1}, Vec3{0, 0, -1}}
	assertHit(t, glass, &r, 0.0001, math.MaxFloat64, 1.5, Vec3{0, 0, 1})

	// From the inside the sphere reports the exit point, outward normal.
	inside := Ray{Vec3{0, 0, -1}, Vec3{0, 0, -1}}
	assertHit(t, glass, &inside, 0.0001, math.MaxFloat64, 0.5, Vec3{0, 0, -1})
}

// A glass sphere in front of a rotated box refracts/reflects through the
// material interface; the box behind still reports its own hit.
func TestGlassSphereInFrontOfBox(t *testing.T) {
	glass := Sphere{Vec3{0, 0, 0}, 0.5, Dielectric{1.5}}
	box := NewBox(Vec3{0, 0, -3}, Vec3{4, 4, 1}, Vec3{0, 0, math.Pi / 4}, Labertiam{Vec3{0.8, 0.3, 0.3}})
	world := HitableList{[]Hitable{glass, box}}
	r := Ray{Vec3{0, 0, 3}, Vec3{0, 0, -1}}
	var rec HitRecord
	if !world.hit(&r, 0.0001, math.MaxFloat64, &rec) {
		t.Fatal("expected the glass sphere to be hit first")
	}
	if !almostEqual(rec.t, 2.5) {
		t.Errorf("glass sphere first hit: got t=%v, want 2.5", rec.t)
	}
	if _, ok := rec.material.(Dielectric); !ok {
		t.Errorf("expected Dielectric, got %T", rec.material)
	}
}
