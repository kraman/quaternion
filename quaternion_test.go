/*
Golang package implementing quaternion math
Purpose is to provide quaternion support under the MIT license as existing
Go quaternion packages are under more restrictive or unspecified licenses.

This project is licensed under the terms of the MIT license.
*/

package quaternion

import (
	"math"
	"testing"
)

var (
	qs1 = Quaternion{1, 0, 0, 0}
	qs2 = Quaternion{W: 10}
	qs3 = Scalar(11)
	qs4 = Scalar(110)
	qv1 = Quaternion{0, 1, 0, 0}
	qv2 = Quaternion{0, 0, 1, 1}
	qv3 = Quaternion{0, 1, 0, 1}
	qv4 = Quaternion{0, 2, 1, 2}
	qv5 = Quaternion{0, -2, -1, -2}
	qv6 = Quaternion{-1, -1, 1, 1}
	q1  = Quaternion{1, -1, -1, 1}
	q2  = Quaternion{-1, 1, 1, -1}
	q0  = Quaternion{0, 0, 0, 0}
	q3  = Quaternion{-1, -1, -1, 1}
	q4  = Quaternion{4, -4, -4, 4}
	q5  = Quaternion{0.5, -0.5, -0.5, 0.5}
	q6  = Quaternion{0.0625, 0.0625, 0.0625, -0.0625}
	q7  = Quaternion{0.24765262787484427, 0.2940044459739585, 0.3943046179925829, 0.8347175749221727}
	q8  = Quaternion{-0.7904669075670613, 0.44891659738265544, -0.3627631346111533, 0.205033813803568}
	q9  = Quaternion{Cos(math.Pi / 2), Sin(math.Pi/2) / Sqrt(3),
		Sin(math.Pi/2) / Sqrt(3), -Sin(math.Pi/2) / Sqrt(3)}
	m = [3][3]float32{[3]float32{-0.333333333, 0.666666667, -0.666666667},
		[3]float32{0.666666667, -0.333333333, -0.666666667},
		[3]float32{-0.666666667, -0.666666667, -0.333333333}}
)

func TestScalarSum(t *testing.T) {
	if Sum(qs1, qs2) != qs3 {
		t.Fail()
	}
}

func TestVectorSum(t *testing.T) {
	if Sum(qv1, qv2, qv3) != qv4 {
		t.Fail()
	}
}

func TestMixedSum(t *testing.T) {
	if Sum(q1, q2) != q0 {
		t.Fail()
	}
}

func TestScalarConj(t *testing.T) {
	if Conj(qs1) != qs1 {
		t.Fail()
	}
}

func TestVectorConj(t *testing.T) {
	if Conj(qv4) != qv5 {
		t.Fail()
	}
}

func TestMixedConj(t *testing.T) {
	if Conj(q2) != q3 {
		t.Fail()
	}
}

func TestScalarProd(t *testing.T) {
	if Prod(qs1, qs2, qs3) != qs4 {
		t.Fail()
	}
}

func TestVectorProd(t *testing.T) {
	if Prod(qv1, qv2, qv3) != qv6 {
		t.Fail()
	}
}

func TestMixedProd(t *testing.T) {
	if Prod(q1, q2, q3) != q4 {
		t.Fail()
	}
}

func TestScalarNorm(t *testing.T) {
	if Norm(qs4) != 110 {
		t.Fail()
	}
}

func TestVectorNorm(t *testing.T) {
	if Norm(qv4) != 3 {
		t.Fail()
	}
}

func TestMixedNorm(t *testing.T) {
	if Norm(q4) != 8 {
		t.Fail()
	}
}

func TestUnit(t *testing.T) {
	if Unit(q4) != q5 {
		t.Fail()
	}
}

func TestInv(t *testing.T) {
	if Inv(q4) != q6 {
		t.Fail()
	}
}

func TestEuler(t *testing.T) {
	phi, theta, psi := Euler(q7)
	if Abs(phi-1.0) > 1e-6 ||
		Abs(theta+0.3) > 1e-6 ||
		Abs(psi-2.4) > 1e-6 {
		t.Fail()
	}
}

func TestFromEuler(t *testing.T) {
	q := FromEuler(-1.2, 0.4, 5.5)
	if Abs(q.W-q8.W) > 1e-6 ||
		Abs(q.X-q8.X) > 1e-6 ||
		Abs(q.Y-q8.Y) > 1e-6 ||
		Abs(q.Z-q8.Z) > 1e-6 {
		t.Fail()
	}

}

func TestRotMat(t *testing.T) {
	mm := RotMat(q9)
	for i, x := range mm {
		for j, y := range x {
			if Abs(m[i][j]-y) > 1e-6 {
				t.Fail()
			}
		}
	}
}
