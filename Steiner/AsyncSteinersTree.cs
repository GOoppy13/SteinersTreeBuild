using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using Deque;
using MyStackLib;
using Steiner.MyThreading;

namespace Steiner
{
    class AsyncSteinersTree
    {
        private static ThreadController tc;
        public static List<KeyValuePair<Point, List<Point>>> BuildSteinersTree(List<Point> inputPoints, int ThreadCount = 1)
        {
            if (inputPoints.Count == 2)
            {
                return new List<KeyValuePair<Point, List<Point>>>()
                {
                    new KeyValuePair<Point, List<Point>>(inputPoints[0], new List<Point>(){ inputPoints[1] }),
                    new KeyValuePair<Point, List<Point>>(inputPoints[1], new List<Point>(){ inputPoints[0] })
                };
            }
            int n = inputPoints.Count / 2;
            List<List<Point>> subsets = new()
            {
                new List<Point>(inputPoints)
            };
            ListPointsEqulityComparer comparer = new();
            for (int i = 1; i <= n; i++)
            {
                foreach (List<Point> list in GetSubsetsOfPoints(i, 0, inputPoints))
                {
                    if (!subsets.Contains(list, comparer))
                    {
                        if (list.Count != 1)
                        {
                            subsets.Add(list);
                        }
                        subsets.Add(inputPoints.Except(list).ToList());
                    }
                }
            }

            List<List<Pair<Point, List<Point>>>> steinersTreeForSubsets = new();
            for (int i = 0; i < subsets.Count; i++)
            {
                steinersTreeForSubsets.Add(null);
            }
            tc = new ThreadController();
            tc.Start(ThreadCount);
            foreach(var list in subsets)
            {
                List<Pair<Point, List<Point>>> tree = BuildSteinersTreeForSubset(list);
                steinersTreeForSubsets[subsets.IndexOf(list)] = tree;
            }
            tc.Stop();
            tc = null;
            return CombinationSubsets(steinersTreeForSubsets, subsets, inputPoints).Select((p) => new KeyValuePair<Point, List<Point>>(p.First, p.Second)).ToList();
        }
        private static List<Pair<Point, List<Point>>> BuildSteinersTreeForSubset(List<Point> subset)
        {
            if (subset.Count == 2)
            {
                return new List<Pair<Point, List<Point>>>()
                {
                    new Pair<Point, List<Point>>(subset[0], new List<Point>() { subset[1] }),
                    new Pair<Point, List<Point>>(subset[1], new List<Point>() { subset[0] })
                };
            }
            List<Pair<Point, List<Point>>> result = null;
            double minLength = double.MaxValue;
            object o = new();
            foreach (var deq in GetReplacementSequences(subset))
            {
                Action action = new Action(() =>
                {
                    double length = ContractionSet(deq, new List<Point>(subset), 
                        out List<Pair<Point, List<Point>>> tmpResult);
                    lock (o)
                    {
                        if (length < minLength)
                        {
                            minLength = length;
                            result = tmpResult;
                        }
                    }
                });
                tc.AddNewTask(action);
            }
            tc.Wait();
            return result;
        }
        private static double ContractionSet(Deque<Pair<int, int>> deq, List<Point> set, out List<Pair<Point, List<Point>>> result)
        {
            MyStack<Pair<Point[], Point?>> st = new();
            List<Point> tmpSubset = new(set);
            result = null;
            double minLength = double.MaxValue;
            do
            {
                if (deq.Count == 0)
                {
                    List<Pair<Point, List<Point>>> tree;
                    try
                    {
                        tree = ReverseCoures(new MyStack<Pair<Point[], Point?>>(st), new List<Point>(tmpSubset));
                    }
                    catch(Exception)
                    {
                        tree = null;
                    }
                    if (tree != null)
                    {
                        double length = GetLengthSteinersTree(tree);
                        if (length < minLength)
                        {
                            minLength = length;
                            result = tree;
                        }
                    }
                    while (st.Count != 0)
                    {
                        Pair<Point[], Point?> t = st.Pop();
                        if (t.Second.HasValue)
                        {
                            st.Push(new Pair<Point[], Point?>(new Point[] { t.First[0], t.First[1], t.Second.Value, t.First[3] }, null));
                            tmpSubset[(int)Math.Min(t.First[3].X, t.First[3].Y)] = t.Second.Value;
                            break;
                        }
                        deq.PushFront(new Pair<int, int>((int)t.First[3].X, (int)t.First[3].Y));
                        tmpSubset.RemoveAt((int)t.First[3].X);
                        tmpSubset.Insert((int)t.First[3].X, t.First[0]);
                        tmpSubset.Insert((int)t.First[3].Y, t.First[1]);
                    }
                    continue;
                }
                else
                {
                    Pair<int, int> indexes = deq.PopFront();
                    Point[] pointsForEquilateralTriangle = GetPointsForEquilateralTriangle(tmpSubset[indexes.First], tmpSubset[indexes.Second]);
                    st.Push(new Pair<Point[], Point?>(new Point[] { tmpSubset[indexes.First], tmpSubset[indexes.Second], pointsForEquilateralTriangle[0], new Point(indexes.First, indexes.Second) },
                        pointsForEquilateralTriangle[1]));
                    tmpSubset.RemoveAt(Math.Max(indexes.First, indexes.Second));
                    tmpSubset.RemoveAt(Math.Min(indexes.First, indexes.Second));
                    tmpSubset.Insert(Math.Min(indexes.First, indexes.Second), pointsForEquilateralTriangle[0]);
                }
            }
            while (st.Count != 0);
            set.Clear();
            deq.Clear();
            return minLength;
        }
        private static IEnumerable<Deque<Pair<int, int>>> GetReplacementSequences(List<Point> subset)
        {
            Deque<Pair<int, int>> deq = new();
            for (int i = 0; i < subset.Count - 1; i++)
            {
                for (int j = i + 1; j < subset.Count; j++)
                {
                    List<Point> tmpSubset = new List<Point>(subset);
                    deq.PushBack(new Pair<int, int>(i, j));
                    tmpSubset.RemoveAt(j);
                    tmpSubset.RemoveAt(i);
                    tmpSubset.Insert(i, new Point());
                    if (tmpSubset.Count == 2)
                    {
                        yield return new Deque<Pair<int, int>>(deq);
                    }
                    else
                    {
                        foreach (Deque<Pair<int, int>> deqq in GetReplacementSequences(tmpSubset))
                        {
                            foreach (Pair<int, int> p in deqq)
                            {
                                deq.PushBack(p);
                            }
                            yield return new Deque<Pair<int, int>>(deq);
                            for (int k = 0; k < deqq.Count; k++)
                            {
                                deq.PopBack();
                            }
                        }
                    }
                    deq.PopBack();
                }
            }
        }
        private static List<Pair<Point, List<Point>>> ReverseCoures(MyStack<Pair<Point[], Point?>> st, List<Point> set)
        {
            List<Pair<Point, List<Point>>> result = new()
            {
                new Pair<Point, List<Point>>(set[0], new List<Point> { set[1] }),
                new Pair<Point, List<Point>>(set[1], new List<Point> { set[0] })
            };
            while (st.Count != 0)
            {
                var points = st.Pop().First;
                int replacePointIndex = -1;
                int connectionPointIndex = -1;
                for (int i = 0; i < result.Count && (replacePointIndex == -1 || connectionPointIndex == -1); i++)
                {
                    int pointIndex = result[i].Second.IndexOf(points[2]);
                    if (pointIndex != -1)
                    {
                        connectionPointIndex = i;
                    }
                    if (result[i].First == points[2])
                    {
                        replacePointIndex = i;
                    }
                }
                double[] circle = GetCircle(points[0], points[1], points[2]);
                Point connectionPoint = result[connectionPointIndex].First;
                if (GetSteinersPoint(circle, connectionPoint, new Point[] { points[0], points[1], points[2] }, out Point steinersPoint))
                {
                    result[replacePointIndex].First = steinersPoint;
                    result[replacePointIndex].Second.Add(points[0]);
                    result[replacePointIndex].Second.Add(points[1]);
                    result[connectionPointIndex].Second[result[connectionPointIndex].Second.IndexOf(points[2])] = steinersPoint;
                    result.Add(new Pair<Point, List<Point>>(points[0], new List<Point>() { steinersPoint }));
                    result.Add(new Pair<Point, List<Point>>(points[1], new List<Point>() { steinersPoint }));
                }
                else
                {
                    return null;
                }
            }
            set.Clear();
            st.Clear();
            return result;
        }
        private static bool GetSteinersPoint(double[] circle, Point connectionPoint, Point[] triangle, out Point steinersPoint)
        {
            if (PointsLocatedSameHalfPlane(new Point(circle[0], circle[1]), connectionPoint, triangle[0], triangle[1]))
            {
                steinersPoint = default;
                return false;
            }
            if ((connectionPoint.X - circle[0]) * (connectionPoint.X - circle[0]) + (connectionPoint.Y - circle[1]) * (connectionPoint.Y - circle[1]) <= circle[2] * circle[2])
            {
                steinersPoint = connectionPoint;
                return true;
            }
            Point[] intersectionPoints = GetIntersectionCircleAndStraightLine(circle, triangle[2], connectionPoint);
            double angle1 = GetAngleBetweenStraightLine(intersectionPoints[0], triangle[0], intersectionPoints[0], triangle[1]);
            double angle2 = GetAngleBetweenStraightLine(intersectionPoints[1], triangle[0], intersectionPoints[1], triangle[1]);
            if (angle1 < 120 && angle2 < 120)
            {
                steinersPoint = default;
                return false;
            }
            steinersPoint = angle1 >= 120 ? intersectionPoints[0] : intersectionPoints[1];
            return true;
        }
        private static Point[] GetIntersectionCircleAndStraightLine(double[] circle, Point A, Point B)
        {
            double k = B.Y - A.Y;
            double h = B.X - A.X;
            Point C1, C2;
            if (h != 0)
            {
                double l = A.X * k - A.Y * h;
                double u = -l / h;
                double t = k / h;
                double a = 1 + t * t;
                double b = 2 * u * t - 2 * circle[0] - 2 * t * circle[1];
                double c = u * u - 2 * u * circle[1] + circle[0] * circle[0] + circle[1] * circle[1] - circle[2] * circle[2];
                double sqrtD = Math.Sqrt(Math.Abs(b * b - 4 * a * c));//Затычка с модулем, подумать, что можно сделать
                double x1 = (-b + sqrtD) / 2 / a;
                double x2 = (-b - sqrtD) / 2 / a;
                double y1 = u + t * x1;
                double y2 = u + t * x2;
                C1 = new Point(x1, y1);
                C2 = new Point(x2, y2);
            }
            else
            {
                double x = A.X;
                double b = -2 * circle[1];
                double c = circle[1] * circle[1] + (x - circle[0]) * (x - circle[0]) - circle[2] * circle[2];
                double sqrtD = Math.Sqrt(Math.Abs(b * b - 4 * c));//Затычка с модулем, подумать, что можно сделать
                double y1 = (-b + sqrtD) / 2;
                double y2 = (-b - sqrtD) / 2;
                C1 = new Point(x, y1);
                C2 = new Point(x, y2);
            }
            return new Point[] { C1, C2 };
        }
        private static double GetAngleBetweenStraightLine(Point A, Point B, Point C, Point D)
        {
            Vector v1 = new(B.X - A.X, B.Y - A.Y);
            Vector v2 = new(D.X - C.X, D.Y - C.Y);
            double cosa = v1 * v2 / v1.Length / v2.Length;
            return Math.Round(Math.Acos(cosa) * 180 / Math.PI, 12);
        }
        private static double[] GetCircle(Point A, Point B, Point C)
        {
            double a = B.X * B.X + B.Y * B.Y - C.X * C.X - C.Y * C.Y;
            double b = C.X * C.X + C.Y * C.Y - A.X * A.X - A.Y * A.Y;
            double c = A.X * A.X + A.Y * A.Y - B.X * B.X - B.Y * B.Y;
            double d = A.X * (B.Y - C.Y) + B.X * (C.Y - A.Y) + C.X * (A.Y - B.Y);
            double x = -1.0 / 2.0 * ((A.Y * a + B.Y * b + C.Y * c) / d);
            double y = 1.0 / 2.0 * ((A.X * a + B.X * b + C.X * c) / d);
            double r = Math.Sqrt((x - A.X) * (x - A.X) + (y - A.Y) * (y - A.Y));
            return new double[] { x, y, r };
        }
        private static bool PointsLocatedSameHalfPlane(Point A, Point B, Point P1, Point P2)
        {
            return GetPointHalfPlane(A, P1, P2) == GetPointHalfPlane(B, P1, P2);
        }
        private static int GetPointHalfPlane(Point A, Point P1, Point P2)
        {
            double a = P2.Y - P1.Y;
            double b = P2.X - P1.X;
            double c = P1.X * a - P1.Y * b;
            if (a * A.X - b * A.Y > c)
                return 1;
            else if (a * A.X - b * A.Y == c)
                return 0;
            else
                return -1;
        }
        private static double GetLengthSteinersTree(List<Pair<Point, List<Point>>> tree)
        {
            if (tree.Count == 2)
            {
                return Math.Sqrt((tree[0].First.X - tree[1].First.X) * (tree[0].First.X - tree[1].First.X) + (tree[0].First.Y - tree[1].First.Y) * (tree[0].First.Y - tree[1].First.Y));
            }
            double length = 0;
            List<Pair<Point, Point>> edges = new();
            foreach (var pair in tree)
            {
                foreach (var point in pair.Second)
                {
                    if (!edges.Exists(p => p.First == pair.First && p.Second == point || p.First == point && p.Second == pair.First))
                    {
                        edges.Add(new Pair<Point, Point>(pair.First, point));
                        length += Math.Sqrt((pair.First.X - point.X) * (pair.First.X - point.X) + (pair.First.Y - point.Y) * (pair.First.Y - point.Y));
                    }
                }
            }
            return length;
        }
        private static Point[] GetPointsForEquilateralTriangle(Point A, Point B)
        {
            double d = (A.X - B.X) * (A.X - B.X) + (A.Y - B.Y) * (A.Y - B.Y);
            double a = B.Y * B.Y + B.X * B.X - A.X * A.X - A.Y * A.Y;
            double b = B.X - A.X;
            double c = B.Y * 2 - A.Y * 2;
            Point C1, C2;
            if (c != 0)
            {
                double r = 1 + 4 * b * b / c / c;
                double p = -2 * A.X + 4 * A.Y * b / c - 4 * a * b / c / c;
                double q = A.X * A.X + A.Y * A.Y - 2 * A.Y * a / c + a * a / c / c - d;
                double sqrtD = Math.Sqrt(Math.Abs(p * p - 4 * r * q));//Затычка с модулем, подумать, что можно сделать
                double x1 = (-p + sqrtD) / (2 * r);
                double x2 = (-p - sqrtD) / (2 * r);
                double y1 = (a - 2 * x1 * b) / c;
                double y2 = (a - 2 * x2 * b) / c;
                C1 = new Point(x1, y1);
                C2 = new Point(x2, y2);
            }
            else
            {
                double x = (B.X * B.X - A.X * A.X) / (2 * (B.X - A.X));
                double p = -2 * B.Y;
                double q = x * x - 2 * B.X * x + B.X * B.X + B.Y * B.Y - d;
                double sqrtD = Math.Sqrt(Math.Abs(p * p - 4 * q));//Затычка с модулем, подумать, что можно сделать
                double y1 = (-p + sqrtD) / 2;
                double y2 = (-p - sqrtD) / 2;
                C1 = new Point(x, y1);
                C2 = new Point(x, y2);
            }
            return new Point[] { C1, C2 };
        }
        private static List<Pair<Point, List<Point>>> CombinationSubsets(List<List<Pair<Point, List<Point>>>> steinersTreeForSubsets, List<List<Point>> subsets, List<Point> originalSet)
        {
            double globMinLength = double.MaxValue;
            List<Pair<Point, List<Point>>> result = null;
            List<double> lengthTrees = new List<double>();
            for (int i = steinersTreeForSubsets.Count - 1; i >= 0; i--)
            {
                if (steinersTreeForSubsets[i] == null)
                {
                    steinersTreeForSubsets.RemoveAt(i);
                    subsets.RemoveAt(i);
                }
            }
            foreach (var tree in steinersTreeForSubsets)
            {
                lengthTrees.Add(GetLengthSteinersTree(tree));
            }
            if (subsets[0].Count == originalSet.Count)
            {
                globMinLength = GetLengthSteinersTree(steinersTreeForSubsets[0]);
                result = steinersTreeForSubsets[0];
                subsets.RemoveAt(0);
                steinersTreeForSubsets.RemoveAt(0);
            }
            int j = 0;
            int n = subsets.Count;
            MyStack<object[]> st = new MyStack<object[]>();
            int k = 0;
            while (j < n)
            {
                while (k < n)
                {
                    List<Point> intersect = subsets[j].Intersect(subsets[k]).ToList();
                    if (intersect.Count == 1 &&
                        steinersTreeForSubsets[j].First(p => p.First == intersect[0]).Second.Count == 1 && steinersTreeForSubsets[k].First(p => p.First == intersect[0]).Second.Count == 1 &&
                        GetAngleBetweenStraightLine(intersect[0], steinersTreeForSubsets[j].First(p => p.First == intersect[0]).Second[0], intersect[0],
                                steinersTreeForSubsets[k].First(p => p.First == intersect[0]).Second[0]) >= 120)
                    {
                        object[] arr = new object[5];
                        arr[0] = subsets;
                        arr[1] = steinersTreeForSubsets;
                        arr[2] = j;
                        arr[3] = k;
                        arr[4] = n;
                        st.Push(arr);
                        subsets = new List<List<Point>>(subsets);
                        var tmpSubset = new List<Point>(subsets[j]);
                        tmpSubset.AddRange(subsets[k]);
                        tmpSubset.Remove(intersect[0]);
                        subsets.RemoveAt(Math.Max(j, k));
                        subsets.RemoveAt(Math.Min(j, k));
                        subsets.Insert(Math.Min(j, k), tmpSubset);
                        steinersTreeForSubsets = new List<List<Pair<Point, List<Point>>>>(steinersTreeForSubsets);
                        var tmpSteinersTreeForSubsets = new List<Pair<Point, List<Point>>>(steinersTreeForSubsets[j]);
                        tmpSteinersTreeForSubsets.AddRange(steinersTreeForSubsets[k]);
                        var p = tmpSteinersTreeForSubsets.First(p => p.First == intersect[0]);
                        tmpSteinersTreeForSubsets.Remove(p);
                        tmpSteinersTreeForSubsets.First(p => p.First == intersect[0]).Second.Add(p.Second[0]);
                        steinersTreeForSubsets.RemoveAt(Math.Max(j, k));
                        steinersTreeForSubsets.RemoveAt(Math.Min(j, k));
                        steinersTreeForSubsets.Insert(Math.Min(j, k), tmpSteinersTreeForSubsets);
                        n = subsets.Count;
                        j = 0;
                        k = 0;
                        continue;
                    }
                    k++;
                }
                j++;
                if (j == n && st.Count != 0)
                {
                    var subset = subsets.FirstOrDefault(s => s.Count == originalSet.Count);
                    if (subset != default)
                    {
                        double length = GetLengthSteinersTree(steinersTreeForSubsets[subsets.IndexOf(subset)]);
                        if (length < globMinLength)
                        {
                            globMinLength = length;
                            result = steinersTreeForSubsets[subsets.IndexOf(subset)];
                        }
                    }
                    var arr = st.Pop();
                    subsets = arr[0] as List<List<Point>>;
                    steinersTreeForSubsets = arr[1] as List<List<Pair<Point, List<Point>>>>;
                    j = Convert.ToInt32(arr[2]);
                    k = Convert.ToInt32(arr[3]);
                    n = Convert.ToInt32(arr[4]);
                    k++;
                    continue;
                }
                k = 0;
            }
            return result;
        }
        public static IEnumerable<List<Point>> GetSubsetsOfPoints(int count, int start, List<Point> points)
        {
            count--;
            for (int i = start; i < points.Count - count; i++)
            {
                List<Point> set = new() { points[i] };
                if (count > 0)
                {
                    foreach (var l in GetSubsetsOfPoints(count, i + 1, points))
                    {
                        yield return set.Concat(l).ToList();
                    }
                }
                else
                {
                    yield return set;
                }
            }
        }
    }
}
