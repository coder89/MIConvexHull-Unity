/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * MIConvexHull, Copyright (c) 2015 David Sehnal, Matthew Campbell
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR.ARFoundation;

namespace MIConvexHull
{
    /// <summary>
    /// Factory class for computing convex hulls.
    /// </summary>
    public static class ConvexHull
    {
        /// <summary>
        /// Creates a convex hull of the input data.
        /// </summary>
        /// <typeparam name="TVertex">The type of the t vertex.</typeparam>
        /// <typeparam name="TFace">The type of the t face.</typeparam>
        /// <param name="data">The data.</param>
        /// <param name="PlaneDistanceTolerance">The plane distance tolerance (default is 1e-10). If too high, points 
        /// will be missed. If too low, the algorithm may break. Only adjust if you notice problems.</param>
        /// <returns>
        /// ConvexHull&lt;TVertex, TFace&gt;.
        /// </returns>
        public static ConvexHullCreationResult<TVertex, TFace> Create<TVertex, TFace>(IList<TVertex> data,
                                                                        double PlaneDistanceTolerance =
                                                                            Constants.DefaultPlaneDistanceTolerance)
            where TVertex : IVertex
            where TFace : ConvexFace<TVertex, TFace>, new()
        {
            return ConvexHull<TVertex, TFace>.Create(data, PlaneDistanceTolerance);
        }

        /// <summary>
        /// Creates a convex hull of the input data.
        /// </summary>
        /// <typeparam name="TVertex">The type of the t vertex.</typeparam>
        /// <param name="data">The data.</param>
        /// <param name="PlaneDistanceTolerance">The plane distance tolerance (default is 1e-10). If too high, points 
        /// will be missed. If too low, the algorithm may break. Only adjust if you notice problems.</param>
        /// <returns>
        /// ConvexHull&lt;TVertex, DefaultConvexFace&lt;TVertex&gt;&gt;.
        /// </returns>
        public static ConvexHullCreationResult<TVertex, DefaultConvexFace<TVertex>> Create<TVertex>(IList<TVertex> data,
                                                                                      double PlaneDistanceTolerance =
                                                                                          Constants.DefaultPlaneDistanceTolerance)
            where TVertex : IVertex
        {
            return ConvexHull<TVertex, DefaultConvexFace<TVertex>>.Create(data, PlaneDistanceTolerance);
        }

        /// <summary>
        /// Creates a convex hull of the input data.
        /// </summary>
        /// <param name="data">The data.</param>
        /// <param name="PlaneDistanceTolerance">The plane distance tolerance (default is 1e-10). If too high, points 
        /// will be missed. If too low, the algorithm may break. Only adjust if you notice problems.</param>
        /// <returns>
        /// ConvexHull&lt;DefaultVertex, DefaultConvexFace&lt;DefaultVertex&gt;&gt;.
        /// </returns>
        public static ConvexHullCreationResult<DefaultVertex, DefaultConvexFace<DefaultVertex>> Create(IList<double[]> data,
                                                                                         double PlaneDistanceTolerance =
                                                                                             Constants.DefaultPlaneDistanceTolerance)
        {
            var points = data.Select(p => new DefaultVertex {Position = p})
                             .ToList();
            return ConvexHull<DefaultVertex, DefaultConvexFace<DefaultVertex>>.Create(points, PlaneDistanceTolerance);
        }

        #region Unity integration

        private static Lazy<Vector2Comparer> vector2Comparer = new Lazy<Vector2Comparer>(() => new Vector2Comparer(), true);
        private static Lazy<Vector3Comparer> vector3Comparer = new Lazy<Vector3Comparer>(() => new Vector3Comparer(), true);

        public static void Create(Vector2[] vertices, List<Vector2> convexPolygon)
        {
            Array.Sort(vertices, vector2Comparer.Value);
            CreateInternal(vertices, convexPolygon);
        }
        public static void Create(List<Vector2> vertices, List<Vector2> convexPolygon)
        {
            vertices.Sort(vector2Comparer.Value);
            CreateInternal(vertices, convexPolygon);
        }
        private static void CreateInternal(IList<Vector2> vertices, List<Vector2> convexPolygon)
        {
            // Based on https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#Java

            if (vertices == null)
            {
                throw new ArgumentNullException("The supplied data is null.");
            }

            if (convexPolygon == null)
            {
                throw new ArgumentNullException(nameof(convexPolygon));
            }

            convexPolygon.Clear();

            var n = vertices.Count;
            var ans = new Vector2[2 * n]; // In between we may have a 2n points
            var k = 0;
            var start = 0;                // start is the first insertion point

            for (var i = 0; i < n; i++)   // Finding lower layer of hull
            {
                var p = vertices[i];
                while (k - start >= 2 && xcross(ans[k - 2], ans[k - 1], p) <= 0)
                    k--;
                ans[k++] = p;
            }

            k--;                         // drop off last point from lower layer
            start = k;

            for (var i = n - 1; i >= 0; i--)                // Finding top layer from hull
            {
                var p = vertices[i];
                while (k - start >= 2 && xcross(ans[k - 2], ans[k - 1], p) <= 0)
                    k--;
                ans[k++] = p;
            }

            k--;                         // drop off last point from top layer

            convexPolygon.Capacity = k + 1;
            convexPolygon.AddRange(ans.Take(k));
        }

        public static void Create(Vector3[] vertices, List<Vector3> convexPolygon)
            => Create(vertices, convexPolygon, null);
        public static void Create(Vector3[] vertices, List<Vector3> convexPolygon, List<int> indices)
        {
            Array.Sort(vertices, vector3Comparer.Value);
            CreateInternal(vertices, convexPolygon, indices);
        }
        public static void Create(List<Vector3> vertices, List<Vector3> convexPolygon, List<int> indices)
        {
            vertices.Sort(vector3Comparer.Value);
            CreateInternal(vertices, convexPolygon, indices);
        }
        private static void CreateInternal(IList<Vector3> vertices, List<Vector3> convexPolygon, List<int> indices)
        {
            // Based on https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#Java

            if (vertices == null)
            {
                throw new ArgumentNullException(nameof(vertices));
            }

            if (convexPolygon == null)
            {
                throw new ArgumentNullException(nameof(convexPolygon));
            }

            convexPolygon.Clear();

            var n = vertices.Count;
            var ans = new Vector3[2 * n]; // In between we may have a 2n points
            var k = 0;
            var start = 0;                // start is the first insertion point

            for (var i = 0; i < n; i++)   // Finding lower layer of hull
            {
                var p = vertices[i];
                while (k - start >= 2 && xcross(ans[k - 2], ans[k - 1], p) <= 0)
                    k--;
                ans[k++] = p;
            }

            k--;                         // drop off last point from lower layer
            start = k;

            for (var i = n - 1; i >= 0; i--)                // Finding top layer from hull
            {
                var p = vertices[i];
                while (k - start >= 2 && xcross(ans[k - 2], ans[k - 1], p) <= 0)
                    k--;
                ans[k++] = p;
            }

            k--;                         // drop off last point from top layer

            convexPolygon.Capacity = k + 1;
            convexPolygon.AddRange(ans.Take(k).Reverse());

            if (indices != null)
            {
                float minX = convexPolygon[0].x, minZ = convexPolygon[0].z;
                float maxX = minX, maxZ = minZ;
                foreach (var point in convexPolygon)
                {
                    if (point.x < minX) minX = point.x;
                    else if (point.x > maxX) maxX = point.x;
                    if (point.z < minZ) minZ = point.z;
                    else if (point.z > maxZ) maxZ = point.z;
                }

                convexPolygon.Add(new Vector3((minX + maxX) / 2.0f, 0, (minZ + maxZ) / 2.0f));

                ARPlaneMeshGenerators.GenerateIndices(indices, convexPolygon);
            }
        }

        private static float xcross(Vector2 O, Vector2 A, Vector2 B)
        {
            return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
        }

        private static float xcross(Vector3 O, Vector3 A, Vector3 B)
        {
            return (A.x - O.x) * (B.z - O.z) - (A.z - O.z) * (B.x - O.x);
        }

        private class Vector2Comparer : IComparer<Vector2>
        {
            int IComparer<Vector2>.Compare(Vector2 p1, Vector2 p2)
            {
                return (p1.x == p2.x)
                    ? p1.y.CompareTo(p2.y)
                    : p1.x.CompareTo(p2.x);
            }
        }

        private class Vector3Comparer : IComparer<Vector3>
        {
            int IComparer<Vector3>.Compare(Vector3 p1, Vector3 p2)
            {
                return (p1.x == p2.x)
                    ? p1.z.CompareTo(p2.z)
                    : p1.x.CompareTo(p2.x);
            }
        }

        #endregion
    }

    /// <summary>
    /// Representation of a convex hull.
    /// </summary>
    /// <typeparam name="TVertex">The type of the t vertex.</typeparam>
    /// <typeparam name="TFace">The type of the t face.</typeparam>
    public class ConvexHull<TVertex, TFace> where TVertex : IVertex
                                            where TFace : ConvexFace<TVertex, TFace>, new()
    {
        /// <summary>
        /// Can only be created using a factory method.
        /// </summary>
        internal ConvexHull()
        {
        }

        /// <summary>
        /// Points of the convex hull.
        /// </summary>
        /// <value>The points.</value>
        public IEnumerable<TVertex> Points { get; internal set; }

        /// <summary>
        /// Faces of the convex hull.
        /// </summary>
        /// <value>The faces.</value>
        public IEnumerable<TFace> Faces { get; internal set; }

        /// <summary>
        /// Creates the convex hull.
        /// </summary>
        /// <param name="data">The data.</param>
        /// <param name="PlaneDistanceTolerance">The plane distance tolerance.</param>
        /// <returns>
        /// ConvexHullCreationResult&lt;TVertex, TFace&gt;.
        /// </returns>
        /// <exception cref="System.ArgumentNullException">The supplied data is null.</exception>
        /// <exception cref="ArgumentNullException">data</exception>
        public static ConvexHullCreationResult<TVertex, TFace> Create(IList<TVertex> data, double PlaneDistanceTolerance)
        {
            if (data == null)
            {
                throw new ArgumentNullException("The supplied data is null.");
            }

            try
            {
                var convexHull = ConvexHullAlgorithm.GetConvexHull<TVertex, TFace>(data, PlaneDistanceTolerance);
                return new ConvexHullCreationResult<TVertex, TFace>(convexHull,ConvexHullCreationResultOutcome.Success);
            }
            catch (ConvexHullGenerationException e)
            {
                return new ConvexHullCreationResult<TVertex, TFace>(null, e.Error, e.ErrorMessage);
            }
            catch (Exception e)
            {
                return new ConvexHullCreationResult<TVertex, TFace>(null, ConvexHullCreationResultOutcome.UnknownError, e.Message);
            }

        }
    }

    public class ConvexHullCreationResult<TVertex, TFace> where TVertex : IVertex
                                                          where TFace : ConvexFace<TVertex, TFace>, new()
    {
        public ConvexHullCreationResult(ConvexHull<TVertex, TFace> result, ConvexHullCreationResultOutcome outcome, string errorMessage="")
        {
            Result  = result;
            Outcome = outcome;
            ErrorMessage = errorMessage;
        }

        //this could be null
        public ConvexHull<TVertex, TFace> Result { get; }

        public ConvexHullCreationResultOutcome Outcome { get; }
        public string ErrorMessage { get; }
    }
}