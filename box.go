package main

import (
	"math"
)

// Box is an axis-aligned box in its own local frame, placed in the world
// with a translation (center) and a rotation (Euler angles in radians,
// applied around X, then Y, then Z). size holds the full edge lengths.
//
// A Box participates in shadows and reflection/refraction like any other
// Hitable: hit fills the record with the closest hit distance, the outward
// (geometric) normal and the box material.
type Box struct {
	center   Vec3
	half     Vec3    // half extents (always >= 0)
	axes     [3]Vec3 // local X/Y/Z axes expressed in world space (orthonormal)
	material Material
	valid    bool // false when constructed with non-finite parameters
}

// NewBox builds a box centered at center with full edge lengths size,
// rotated by the Euler angles rotation (radians, X then Y then Z order).
// Negative sizes are treated as their absolute value. Any non-finite
// parameter yields an invalid box that never reports a hit.
func NewBox(center Vec3, size Vec3, rotation Vec3, material Material) Box {
	b := Box{center: center, material: material, valid: true}
	if !isFiniteVec(center) || !isFiniteVec(size) || !isFiniteVec(rotation) {
		b.valid = false
		return b
	}
	b.half = Vec3{math.Abs(size.X) / 2, math.Abs(size.Y) / 2, math.Abs(size.Z) / 2}

	cx, sx := math.Cos(rotation.X), math.Sin(rotation.X)
	cy, sy := math.Cos(rotation.Y), math.Sin(rotation.Y)
	cz, sz := math.Cos(rotation.Z), math.Sin(rotation.Z)

	// R = Rz * Ry * Rx; its columns are the local axes in world space.
	b.axes[0] = Vec3{cy * cz, cy * sz, -sy}
	b.axes[1] = Vec3{cz*sy*sx - sz*cx, sz*sy*sx + cz*cx, cy * sx}
	b.axes[2] = Vec3{cz*sy*cx + sz*sx, sz*sy*cx - cz*sx, cy * cx}
	return b
}

func isFiniteVec(v Vec3) bool {
	return isFinite(v.X) && isFinite(v.Y) && isFinite(v.Z)
}

func isFinite(f float64) bool {
	return !math.IsNaN(f) && !math.IsInf(f, 0)
}

// component returns the i-th component of v (0=X, 1=Y, 2=Z).
func component(v Vec3, i int) float64 {
	switch i {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

func (b Box) hit(r *Ray, tMin float64, tMax float64, rec *HitRecord) bool {
	if !b.valid {
		return false
	}
	// Transform the ray into the box local frame (rotation is orthonormal,
	// so distances and the t parameter are preserved).
	oc := r.Origin().Substract(b.center)
	lo := Vec3{b.axes[0].Dot(oc), b.axes[1].Dot(oc), b.axes[2].Dot(oc)}
	ld := Vec3{b.axes[0].Dot(r.Direction()), b.axes[1].Dot(r.Direction()), b.axes[2].Dot(r.Direction())}

	// Slab test. tNear/tFar bracket the segment of the ray inside the box.
	tNear := tMin
	tFar := tMax
	nearAxis, nearSign := -1, 1.0
	farAxis, farSign := -1, 1.0
	for i := 0; i < 3; i++ {
		o := component(lo, i)
		d := component(ld, i)
		h := component(b.half, i)
		inv := 1.0 / d // ±Inf for a parallel ray keeps the comparisons meaningful
		t1 := (-h - o) * inv
		t2 := (h - o) * inv
		sign := -1.0 // t1 plane is -h, outward normal -e_i
		if t1 > t2 {
			t1, t2 = t2, t1
			sign = 1.0
		}
		// Strict > gives a deterministic tie-break on edge/corner hits:
		// the lowest-index axis wins.
		if t1 > tNear {
			tNear = t1
			nearAxis = i
			nearSign = sign
		}
		if t2 < tFar {
			tFar = t2
			farAxis = i
			farSign = -sign
		}
		if tNear > tFar {
			return false
		}
	}

	var t float64
	var axis int
	var sign float64
	switch {
	case nearAxis >= 0:
		// Ray enters the box from outside.
		t, axis, sign = tNear, nearAxis, nearSign
	case farAxis >= 0 && tFar > tMin:
		// Ray starts inside the box (or on a face): report the exit face.
		t, axis, sign = tFar, farAxis, farSign
	default:
		return false
	}

	localNormal := Vec3{}
	switch axis {
	case 0:
		localNormal.X = sign
	case 1:
		localNormal.Y = sign
	default:
		localNormal.Z = sign
	}

	rec.t = t
	rec.p = r.PointAtParameter(t)
	// Rotate the outward normal back to world space.
	rec.normal = b.axes[0].ScalarMultiple(localNormal.X).
		Add(b.axes[1].ScalarMultiple(localNormal.Y)).
		Add(b.axes[2].ScalarMultiple(localNormal.Z))
	rec.material = b.material
	return true
}
