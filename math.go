package quaternion

// #include <math.h>
import "C"

func Sqrt(n float32) float32 {
	return float32(C.sqrt(C.double(n)))
}

func Atan2(a1 float32, a2 float32) float32 {
	return float32(C.atan2(C.double(a1), C.double(a2)))
}

func Asin(a1 float32) float32 {
	return float32(C.asin(C.double(a1)))
}

func Cos(a1 float32) float32 {
	return float32(C.cos(C.double(a1)))
}

func Sin(a1 float32) float32 {
	return float32(C.sin(C.double(a1)))
}

func Abs(a1 float32) float32 {
	return float32(C.fabs(C.double(a1)))
}
