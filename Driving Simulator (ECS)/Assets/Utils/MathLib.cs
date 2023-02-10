using System;
using UnityEngine;

namespace Utils
{
    // Much of code comes from BeamNG.drive's lua/common/mathlib.lua
    public static class MathLib
    {
        // Source: https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
        // Adapted to 3D and different use case 
        public class CatmullRomCurve
        {
            public struct CurvePoint
            {
                public Vector3 point;
                public Vector3 dir;
                public Vector3 up;
                public Vector3 left;
                public float radius;
            }

            public Vector3 p0, p1, p2, p3;
            public float alpha;

            public CatmullRomCurve(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float alpha = 0.5f)
            {
                (this.p0, this.p1, this.p2, this.p3) = (p0, p1, p2, p3);
                this.alpha = alpha;
            }

            // Evaluates a point at the given t-value from 0 to 1
            public Vector3 GetPoint(float t)
            {
                // calculate knots
                const float k0 = 0;
                float k1 = GetKnotInterval(p0, p1);
                float k2 = GetKnotInterval(p1, p2) + k1;
                float k3 = GetKnotInterval(p2, p3) + k2;

                // evaluate the point
                float u = Mathf.LerpUnclamped(k1, k2, t);
                Vector3 A1 = Remap(k0, k1, p0, p1, u);
                Vector3 A2 = Remap(k1, k2, p1, p2, u);
                Vector3 A3 = Remap(k2, k3, p2, p3, u);
                Vector3 B1 = Remap(k0, k2, A1, A2, u);
                Vector3 B2 = Remap(k1, k3, A2, A3, u);
                return Remap(k1, k2, B1, B2, u);
            }

            // Gets the closest point on the curve represented as 'numLines' line segments
            public Vector3 ClosestPointOnCurve(Vector3 pos, int numLines)
            {
                for (int i = 0; i < numLines; i++)
                {
                    Vector3 start = GetPoint(i / (float)numLines);
                    Vector3 mid = GetPoint((i + 0.5f) / (float)numLines);
                    Vector3 end = GetPoint((i + 1) / (float)numLines);
                    if (start == end) continue;

                    // Check if 'pos' being projected on line segment falls in line segment bounds
                    // If xnorm is < 0 on first line segment or > 1 on last line segment,
                    // it falls outside curve completely and just return then
                    float xnorm = InverseLerp(pos, start, end);
                    if (
                        (i == 0 && xnorm < 0.0f) ||
                        (i == numLines - 1 && xnorm > 1.0f) ||
                        (xnorm >= 0.0f && xnorm <= 1.0f))
                    {
                        //Debug.Log(i + ", " + xnorm);
                        return Vector3.LerpUnclamped(start, end, xnorm);
                    }
                }

                return Vector3.zero;
            }

            // Gets the closest point on the curve represented as 'numLines' line segments
            public CurvePoint ClosestPointOnCurveMoreInfo(Vector3 pos, int numLines)
            {
                CurvePoint ret = new CurvePoint();

                for (int i = 0; i < numLines; i++)
                {
                    Vector3 start = GetPoint(i / (float) numLines);
                    Vector3 mid = GetPoint((i + 0.5f) / (float)numLines);
                    Vector3 end = GetPoint((i + 1) / (float) numLines);
                    if (start == end) continue;

                    // Check if 'pos' being projected on line segment falls in line segment bounds
                    // If xnorm is < 0 on first line segment or > 1 on last line segment,
                    // it falls outside curve completely and just return then
                    float xnorm = InverseLerp(pos, start, end);                  
                    if (
                        (i == 0 && xnorm < 0.0f) ||
                        (i == numLines - 1 && xnorm > 1.0f) ||
                        (xnorm >= 0.0f && xnorm <= 1.0f))
                    {
                        //Debug.Log(i + ", " + xnorm);
                        ret.point = Vector3.LerpUnclamped(start, end, xnorm);
                        ret.dir = (end - start).normalized;
                        ret.up = Vector3.up;
                        ret.left = Vector3.Cross(ret.dir, ret.up).normalized;
                        ret.radius = MathLib.GetRadius(start, mid, end);
                        break;
                    }
                }

                return ret;
            }

            static Vector3 Remap(float a, float b, Vector3 c, Vector3 d, float u)
            {
                return Vector3.LerpUnclamped(c, d, (u - a) / Mathf.Max(1e-30f, b - a));
            }

            float GetKnotInterval(Vector3 a, Vector3 b)
            {
                return Mathf.Pow((a - b).sqrMagnitude, 0.5f * alpha);
            }
        }

        public class CatmullRomSpline
        {
            public Vector3[] points;
            public float alpha;
            public int pointCount;
            public int segmentCount;

            public CatmullRomSpline(Vector3[] points, float alpha)
            {
                this.points = points;
                this.alpha = alpha;
                this.pointCount = points.Length;
                this.segmentCount = this.pointCount - 1;
            }

            public CatmullRomCurve GetCurve(int i)
            {
                Vector3 p0 = i - 1 >= 0 ? points[i - 1] : points[0];
                Vector3 p3 = i + 2 < points.Length ? points[i + 2] : points[pointCount - 1];

                return new CatmullRomCurve(p0, points[i], points[i + 1], p3, alpha);
            }
        }

        // From lua/common/filters.lua
        public class ExponentialSmoother
        {
            float a;
            float startingValue;
            float st;

            public ExponentialSmoother(int window, float startingValue = 0.0f, float fixedDt = 0.02f)
            {
                this.a = 2.0f / Mathf.Max(window, 2);
                this.startingValue = startingValue;
                this.st = startingValue;
                float dt = fixedDt;
                float adt = this.a * dt;
                this.a = (1.0f / dt + this.a) * adt / (1 + adt);
            }

            public float Get(float sample)
            {
                this.st += this.a * (sample - this.st);
                return st;
            }
        }

        public static float InverseLerp(Vector3 pos, Vector3 a, Vector3 b)
        {
            float bax = b.x - a.x, bay = b.y - a.y, baz = b.z - a.z;
            return (bax * (pos.x - a.x) + bay * (pos.y - a.y) + baz * (pos.z - a.z)) / (bax * bax + bay * bay + baz * baz + 1e-30f);
        }

        public static float IntersectsRayPlane(Vector3 rpos, Vector3 rdir, Vector3 plpos, Vector3 pln)
        {
            return Mathf.Min(Vector3.Dot(plpos - rpos, pln) / Vector3.Dot(rdir, pln), float.MaxValue);
        }

        public static float GetRadius(Vector3 wp1, Vector3 wp2, Vector3 wp3)
        {
            Vector3 vec1 = wp2 - wp1;
            Vector3 vec2 = wp3 - wp2;
            Vector3 vec1Left = Vector3.Cross(vec1, Vector3.up);

            return IntersectsRayPlane(wp2, vec1Left.normalized, wp3, vec2.normalized);
        }

        public static float GetCurvature(Vector3 vec1, Vector3 vec2)
        {
            float vec1Sqlen = vec1.sqrMagnitude, vec2Sqlen = vec2.sqrMagnitude;
            float dot12 = Vector3.Dot(vec1, vec2);
            float cos8sq = Mathf.Min(1, dot12 * dot12 / Mathf.Max(1e-30f, vec1Sqlen * vec2Sqlen));

            if (dot12 < 0)
            {
                float minDsq = Mathf.Min(vec1Sqlen, vec2Sqlen);
                float maxDsq = minDsq / Mathf.Max(1e-30f, cos8sq);
                if (Mathf.Max(vec1Sqlen, vec2Sqlen) > (minDsq + maxDsq) * 0.5f)
                {
                    if (vec1Sqlen > vec2Sqlen)
                    {
                        Vector3 tempVec = vec1;
                        vec1 = vec2;
                        vec2 = tempVec;

                        float tempLen = vec1Sqlen;
                        vec1Sqlen = vec2Sqlen;
                        vec2Sqlen = tempLen;
                    }
                    vec2 *= Mathf.Sqrt(0.5f * (minDsq + maxDsq) / Mathf.Max(1e-30f, vec2Sqlen));
                }
            }

            vec2 *= -1f;
            return 2 * Mathf.Sqrt((1 - cos8sq) / Mathf.Max(1e-30f, (vec1 - vec2).sqrMagnitude));
        }

    }
}