using System;

namespace rt
{
    public class RayTracer
    {
        private Geometry[] geometries;
        private Light[] lights;

        public RayTracer(Geometry[] geometries, Light[] lights)
        {
            this.geometries = geometries;
            this.lights = lights;
        }

        private double ImageToViewPlane(int n, int imgSize, double viewPlaneSize)
        {
            return -n * viewPlaneSize / imgSize + viewPlaneSize / 2;
        }

        private Intersection FindFirstIntersection(Line ray, double minDist, double maxDist)
        {
            var intersection = Intersection.NONE;

            foreach (var geometry in geometries)
            {
                var intr = geometry.GetIntersection(ray, minDist, maxDist);

                if (!intr.Valid || !intr.Visible) continue;

                if (!intersection.Valid || !intersection.Visible)
                {
                    intersection = intr;
                }
                else if (intr.T < intersection.T)
                {
                    intersection = intr;
                }
            }
            
            return intersection;
        }

        private bool IsLit(Vector point, Light light)
        {

            var ray = new Line(light.Position, point);
            var intersection = FindFirstIntersection(ray, 0, 1000000);
            if (!intersection.Valid || !intersection.Visible)
            {
                return true;
            }
            return intersection.T > (light.Position - point).Length() - 0.001;
        }

        public void Render(Camera camera, int width, int height, string filename)
        {
            var background = new Color(0.6, 0.2, 0.2, 1.0);

            var viewParallel = (camera.Up ^ camera.Direction).Normalize();

            var image = new Image(width, height);

            var vecW = camera.Direction * camera.ViewPlaneDistance;
            for (var i = 0; i < width; i++)
            {
                for (var j = 0; j < height; j++)
                {
                    var pointOnViewPlane = camera.Position + vecW +
                                           camera.Up * ImageToViewPlane(j, height, camera.ViewPlaneHeight) +
                                           viewParallel * ImageToViewPlane(i, width, camera.ViewPlaneWidth);
                    var ray = new Line(camera.Position, pointOnViewPlane);
                    var intersection = FindFirstIntersection(ray, camera.FrontPlaneDistance, camera.BackPlaneDistance);
                    if (intersection.Valid && intersection.Visible)
                    {
                        var color = new Color();
                        foreach (var light in lights)
                        {
                            var lightColor = new Color();
                            lightColor += intersection.Material.Ambient * light.Ambient;
                            if (IsLit(intersection.Position, light))
                            {
                                var t = (light.Position - intersection.Position).Normalize();
                                var n = intersection.Normal;
                                var e = (camera.Position - intersection.Position).Normalize();
                                var r = (n * (n * t) * 2 - t).Normalize();
                                if (n * t > 0)
                                {
                                    lightColor += intersection.Material.Diffuse * light.Diffuse * (n * t);
                                }

                                if (e * r > 0)
                                {
                                    lightColor += intersection.Material.Specular * light.Specular *
                                                  Math.Pow(e * r, intersection.Material.Shininess);
                                }

                            }
                            color += lightColor;
                        }
                        image.SetPixel(i, j, color);
                    }
                    else
                    {
                        image.SetPixel(i, j, background);
                    }
                }
            }

            image.Store(filename);
        }
    }
}