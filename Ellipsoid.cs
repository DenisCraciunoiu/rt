using System;
using System.Linq.Expressions;


namespace rt
{
    public class Ellipsoid : Geometry
    {

        public struct Matrix3x3
        {
            public double M11, M12, M13;
            public double M21, M22, M23;
            public double M31, M32, M33;

            public Matrix3x3(
                double m11, double m12, double m13,
                double m21, double m22, double m23,
                double m31, double m32, double m33)
            {
                M11 = m11; M12 = m12; M13 = m13;
                M21 = m21; M22 = m22; M23 = m23;
                M31 = m31; M32 = m32; M33 = m33;
            }

            // Transpose
            public Matrix3x3 Transpose()
            {
                return new Matrix3x3(
                    M11, M21, M31,
                    M12, M22, M32,
                    M13, M23, M33
                );
            }

            // Matrix * Matrix
            public static Matrix3x3 operator *(Matrix3x3 a, Matrix3x3 b)
            {
                return new Matrix3x3(
                    a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31,
                    a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32,
                    a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33,

                    a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31,
                    a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32,
                    a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33,

                    a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31,
                    a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32,
                    a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33
                );
            }

            // Matrix * Vector
            public static Vector operator *(Matrix3x3 m, Vector v)
            {
                return new Vector(
                    m.M11 * v.X + m.M12 * v.Y + m.M13 * v.Z,
                    m.M21 * v.X + m.M22 * v.Y + m.M23 * v.Z,
                    m.M31 * v.X + m.M32 * v.Y + m.M33 * v.Z
                );
            }

            // For readability
            public override string ToString()
            {
                return $"[{M11}, {M12}, {M13}; {M21}, {M22}, {M23}; {M31}, {M32}, {M33}]";
            }
        }

        private Vector Center { get; }
        private Vector SemiAxesLength { get; }
        private double Radius { get; }

        public Quaternion Rotation { get; set; } = Quaternion.NONE; 

        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Material material, Color color) : base(material, color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Color color) : base(color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public Ellipsoid(Ellipsoid e) : this(new Vector(e.Center), new Vector(e.SemiAxesLength), e.Radius, new Material(e.Material), new Color(e.Color)) 
        {
        }

        private Vector Normal(Vector center, double A, double B, double C, Vector point)
        {
            return new Vector(
                2 * (point.X - center.X) / (A * A),
                2 * (point.Y - center.Y) / (B * B),
                2 * (point.Z - center.Z) / (C * C)
            ).Normalize();
        }

        public override Intersection GetIntersection(Line line, double minDist, double maxDist)
        {
            // --- Helper: 3×3 matrix and vector operations ---
            static Vector Multiply(Matrix3x3 M, Vector v)
            {
                return new Vector(
                    M.M11 * v.X + M.M12 * v.Y + M.M13 * v.Z,
                    M.M21 * v.X + M.M22 * v.Y + M.M23 * v.Z,
                    M.M31 * v.X + M.M32 * v.Y + M.M33 * v.Z
                );
            }

            static double Dot(Vector a, Vector b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

            // --- Convert quaternion rotation to 3×3 matrix ---
            Quaternion q = this.Rotation;
            double w = q.W, x = q.X, y = q.Y, z = q.Z;

            
            Matrix3x3 R = new Matrix3x3(
                1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w),
                2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)
            );

            // --- Construct ellipsoid quadratic form M = R * D * R^T ---
            double invA2 = 1.0 / (SemiAxesLength.X * SemiAxesLength.X);
            double invB2 = 1.0 / (SemiAxesLength.Y * SemiAxesLength.Y);
            double invC2 = 1.0 / (SemiAxesLength.Z * SemiAxesLength.Z);

            Matrix3x3 D = new Matrix3x3(
                invA2, 0, 0,
                0, invB2, 0,
                0, 0, invC2
            );

            Matrix3x3 Rt = R.Transpose();
            Matrix3x3 M = R * D * Rt; // final quadratic form representing the rotated ellipsoid

            // --- Compute intersection quadratic coefficients ---
            Vector o = line.X0 - Center; // origin relative to ellipsoid center
            Vector d = line.Dx;

            Vector M_d = Multiply(M, d);
            Vector M_o = Multiply(M, o);

            double A = Dot(d, M_d);
            double B = 2.0 * Dot(d, M_o);
            double C = Dot(o, M_o) - Radius * Radius;

            double delta = B * B - 4 * A * C;
            if (delta < 0.0001)
                return new Intersection(false, false, this, line, 0, null, Material, Color);

            double sqrtDelta = Math.Sqrt(delta);
            double t1 = (-B - sqrtDelta) / (2 * A);
            double t2 = (-B + sqrtDelta) / (2 * A);

            bool t1ok = minDist <= t1 && t1 <= maxDist;
            bool t2ok = minDist <= t2 && t2 <= maxDist;

            double t;
            if (!t1ok && !t2ok)
                return new Intersection(false, false, this, line, 0, null, Material, Color);
            else if (t1ok && (!t2ok || t1 < t2))
                t = t1;
            else
                t = t2;

            // --- Intersection point ---
            Vector p = line.X0 + d * t;

            // --- Normal: gradient of implicit function F(x) = x^T M x - R^2 ---
            Vector grad = Multiply(M, p - Center) * 2.0;
            Vector normal = grad.Normalize();

            return new Intersection(true, true, this, line, t, normal, Material, Color);
        }



    }
}
