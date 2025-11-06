using System;
using System.Linq.Expressions;


namespace rt
{
    public class Ellipsoid : Geometry
    {
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
            var a = line.Dx.X;
            var b = line.X0.X;
            var c = line.Dx.Y;
            var d = line.X0.Y;
            var e = line.Dx.Z;
            var f = line.X0.Z;

            var A = SemiAxesLength.X * SemiAxesLength.X;
            var B = SemiAxesLength.Y * SemiAxesLength.Y;
            var C = SemiAxesLength.Z * SemiAxesLength.Z;

            var ap = a * a / A + c * c / B + e * e / C;
            var bp = 2 * (a * (b - Center.X) / A + c * (d - Center.Y) / B + e * (f - Center.Z) / C);
            var cp = (b - Center.X) * (b - Center.X) / A + (d - Center.Y) * (d - Center.Y) / B + (f - Center.Z) * (f - Center.Z) / C - Radius * Radius;
            var delta = bp * bp - 4 * ap * cp;
            if (delta < 0.0001)
            {
                return new Intersection(false, false, this, line, 0, null, this.Material, this.Color);
            }
            var t1 = (-bp - Math.Sqrt(bp * bp - 4 * ap * cp)) / (2 * ap);
            var t2 = (-bp + Math.Sqrt(bp * bp + 4 * ap * cp)) / (2 * ap);

            var t1verif = minDist <= t1 && t1 <= maxDist;
            var t2verif = minDist <= t2 && t2 <= maxDist;

            if (t1verif == false && t2verif == false)
                return new Intersection(false, false, this, line, 0, null, this.Material, this.Color);
            if (t1verif == false && t2verif == true)
                return new Intersection(true, true, this, line, t2, this.Normal(Center, SemiAxesLength.X, SemiAxesLength.Y, SemiAxesLength.Z, line.Dx * t2 + line.X0), this.Material, this.Color);
            if (t2verif == false && t1verif == true)
                return new Intersection(true, true, this, line, t1, this.Normal(Center, SemiAxesLength.X, SemiAxesLength.Y, SemiAxesLength.Z, line.Dx * t1 + line.X0), this.Material, this.Color);

            if (t1 < t2)
                return new Intersection(true, true, this, line, t1, this.Normal(Center, SemiAxesLength.X, SemiAxesLength.Y, SemiAxesLength.Z, line.Dx * t1 + line.X0), this.Material, this.Color);
            return new Intersection(true, true, this, line, t2, this.Normal(Center, SemiAxesLength.X, SemiAxesLength.Y, SemiAxesLength.Z, line.Dx * t2 + line.X0), this.Material, this.Color);
        }
    }
}
