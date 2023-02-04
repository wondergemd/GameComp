using System;
using UnityEngine;

namespace Utils
{
    // Much of code comes from BeamNG.drive's lua/common/mathlib.lua
    public static class MathLib
    {
        // From lua/common/filters.lua
        public class ExponentialSmoother
        {
            float a;
            float startingValue;
            float st;

            public ExponentialSmoother(int window, float? startingValue, float? fixedDt)
            {
                this.a = 2.0f / Mathf.Max(window, 2);
                this.startingValue = startingValue ?? 0;
                this.st = startingValue ?? 0;
                float dt = fixedDt ?? 0.02f;
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