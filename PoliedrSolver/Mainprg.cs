using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PriorityQueueDemo;

namespace PoliedrSolver
{
    class Mainprg
    {
        private List<List<Edge>> nodesList;
        private int[] distances;
        private int n, m;
        private StreamWriter sw;

        private double[] ways;
        private int[] pred;

        private List<int>[] nodes;
        private List<FlowEdge> fEdges;
        private int[] ptr;
        private Queue<int> queue;
        private int s, t;

        static void Main(string[] args)
        {
            new Mainprg().Execute(args);
        }

        public void Execute(string[] args)
        {
            //InitializeGraphFromTestFile(args[0]);
            Console.WriteLine(args[1]);
            InitializeGraphFromBrsFile(args[0]);
            if (args[1] == "2")
                SolveMinimalTreeTask(int.Parse(args[2]));
            else if (args[1] == "3")
                SolveShortestWayTask(int.Parse(args[2]), int.Parse(args[3]));
            else if (args[1] == "4")
                SolveMaxFlowTask(int.Parse(args[2]), int.Parse(args[3]));
            sw.Flush();
            sw.Close();
        }

        private void SolveMaxFlowTask(int s, int t)
        {
            Console.WriteLine("Starting max flow solving...");
            this.s = s;
            this.t = t;
            nodes = new List<int>[n];
            for (int i = 0; i < n; i++)
                nodes[i] = new List<int>(6);
            fEdges = new List<FlowEdge>(2 * n);
            ptr = new int[n];
            queue = new Queue<int>();
            
            foreach (var node in nodesList)
                foreach (var edge in node)
                {
                    AddFlowEdge(edge.From, edge.To, edge.Weight);
                }

            double flow = 0;
            for (;;)
            {
                if (!Bfs())
                    break;
                Array.Clear(ptr, 0, ptr.Length);
                double pushed = 0;
                while ((pushed = Dfs(s, double.MaxValue)) > 0)
                {
                    flow += pushed;
                    //Console.WriteLine($"{flow} {pushed}");
                }
            }

            Console.WriteLine("Max flow: " + flow);

            int[] colors = distances;
            Array.Clear(colors, 0, colors.Length);
            foreach (var edge in fEdges)
            {
                if (edge.Flow > 1e-6)
                {
                    //flows[edge.From] += edge.Flow;
                    colors[edge.To] = colors[edge.From] = 1;
                }
            }

            colors[t] = colors[s] = 2;
            sw.WriteLine("\nparts 255");
            for (int i = 0; i < colors.Length; i++)
                sw.WriteLine(colors[i]);
        }

        private bool Bfs()
        {
            queue.Clear();
            queue.Enqueue(s);
            for (int i = 0; i < distances.Length; i++)
                distances[i] = -1;
            distances[s] = 0;
            //Console.WriteLine(distances[t]);
            while (queue.Count != 0 && distances[t] == -1)
            {
                int v = queue.Dequeue();
                for (int i = 0; i < nodes[v].Count; i++)
                {
                    int id = nodes[v][i],
                        to = fEdges[id].To;
                    if (distances[to] == -1 && (fEdges[id].Cap - fEdges[id].Flow > 1e-6))
                    {
                        //Console.WriteLine("dist:" + distances[v]);
                        //Console.WriteLine(fEdges[id].Cap + " " + fEdges[id].Flow);
                        queue.Enqueue(to);
                        distances[to] = distances[v] + 1;
                    }
                }
            }
            //Console.WriteLine(distances[t] + " " + t);
            return distances[t] != -1;
        }

        private double Dfs(int v, double flow)
        {
            if (flow <= 0)
                return 0;
            if (v == t)
                return flow;
            for (; ptr[v] < nodes[v].Count; ptr[v]++)
            {
                int id = nodes[v][ptr[v]],
                    to = fEdges[id].To;
                if (distances[to] != distances[v] + 1)
                    continue;
                double pushed = Dfs(to, Math.Min(flow, fEdges[id].Cap - fEdges[id].Flow));
                if (pushed > 0)
                {
                    fEdges[id].Flow += pushed;
                    fEdges[id ^ 1].Flow -= pushed;
                    return pushed;
                }
            }
            return 0;
        }

        private void AddFlowEdge(int from, int to, double cap)
        {
            FlowEdge e1 = new FlowEdge(from, to, cap, 0);
            FlowEdge e2 = new FlowEdge(to, from, 0, 0);
            nodes[from].Add(fEdges.Count);
            fEdges.Add(e1);
            nodes[to].Add(fEdges.Count);
            fEdges.Add(e2);
        }



        private void SolveShortestWayTask(int from, int to)
        {
            FindMinimalTree(from);
            int count = 0;
            int dest = 0;
            pred = new int[n];
            ways = new double[n];

            for (int i = 0; i < distances.Length; i++)
            {
                ways[i] = double.MaxValue;
                pred[i] = i;
                if (distances[i] != -1)
                {
                    count++;
                    if (count == to)
                    {
                        dest = i;
                    }
                }
                distances[i] = 0;
            }
            int[] colors = distances;
            Console.WriteLine(from + " " + dest);
            FindShortestWays(from, dest);
            int cur = dest;
            do
            {
                colors[cur] = 1;
                cur = pred[cur];
            } while (pred[cur] != cur);
            colors[from] = 1;
            Console.WriteLine("Count: " + count);
            sw.WriteLine("\nparts 255");
            count = 0;
            for (int i = 0; i < colors.Length; i++)
            {
                sw.WriteLine(colors[i]);
                if (colors[i] == 1)
                {
                    count++;
                }
            }
            Console.WriteLine("Distance: " + count);
            Console.WriteLine("Total way: " + ways[dest]);
        }

        private void FindShortestWays(int from, int to)
        {
            ways[from] = 0;
            PriorityQueue<double, int> nodes = new PriorityQueue<double, int>(n);
            bool[] isVisited = new bool[n];
            nodes.Enqueue(0, from);

            while (!nodes.IsEmpty)
            {
                int cur = nodes.Dequeue().Value;
                //Console.WriteLine(cur);
                if (cur == to)
                    break;
                if (isVisited[cur])
                    continue;
                isVisited[cur] = true;
                foreach (Edge edge in nodesList[cur])
                {
                    double cost = ways[cur] + edge.Weight;
                    if (!isVisited[edge.To] && (cost < ways[edge.To]))
                    {
                        ways[edge.To] = cost;
                        pred[edge.To] = cur;
                        nodes.Enqueue(cost, edge.To);
                    }
                }
            }
        }

        private void SolveMinimalTreeTask(int from)
        {
            Console.WriteLine(
                        $"Weight: { FindMinimalTree(from).ToString(CultureInfo.InvariantCulture.NumberFormat)}");
            sw.WriteLine("\nparts 255 ;; number of parts");
            for (int i = 0; i < distances.Length; i++)
            {
                sw.WriteLine(distances[i] == -1 ? "254" : Math.Min(distances[i], 254).ToString());
                //if (distances[i] == -1)
                //{
                //    Console.WriteLine(
                //        $"Weight: { FindMinimalTree(i).ToString(CultureInfo.InvariantCulture.NumberFormat)}");
                //}
            }
            //for (int i = 0; i < n; i++)
            //    Console.WriteLine($"{i}: {distances[i]}");
            //PrintGraph(nodesList);
        }

        public double FindMinimalTree(int startNode)
        {
            bool[] isVisited = new bool[n];
            PriorityQueue<double, Edge> edges = new PriorityQueue<double, Edge>(n);
            isVisited[startNode] = true;
            distances[startNode] = 0;
            AddEdges(edges, startNode, isVisited);
            double totalWeight = 0;
            while (edges.Count != 0)
            {
                Edge currentEdge = edges.Dequeue().Value;
                if (isVisited[currentEdge.To])
                    continue;
                totalWeight += currentEdge.Weight;
                distances[currentEdge.To] = distances[currentEdge.From] + 1;
                isVisited[currentEdge.To] = true;
                AddEdges(edges, currentEdge.To, isVisited);
                //Console.WriteLine(currentEdge);
            }

            return totalWeight;
        }

        private void AddEdges(PriorityQueue<double, Edge> edges, int nodeNumber, bool[] isVisited)
        {
            foreach (var edge in nodesList[nodeNumber])
            {
                //Console.WriteLine("POSSIBLE ADD: " + edge);
                if (!isVisited[edge.To])
                {
                    //Console.WriteLine("ADDED: " + edge);
                    edges.Enqueue(edge.Weight, edge);
                }
            }    
        }

        public void PrintGraph(List<List<Edge>> graph)
        {
            foreach (var list in graph)
            {
                foreach (var edge in list)
                {
                    Console.WriteLine(edge);
                }
                Console.WriteLine();
            }
        }

        public void InitializeGraphFromBrsFile(string filename)
        {
            sw = new StreamWriter(new FileStream("result.brs", FileMode.Create));
            using (StreamReader sr = new StreamReader(new FileStream(filename, FileMode.Open)))
            {
                string line;
                for (int i = 0; i < 3; i++)
                {
                    line = sr.ReadLine();
                    sw.WriteLine(line);
                }
                line = sr.ReadLine();
                sw.WriteLine(line);
                string strPointsNum = line.Split(' ')[1];
                int pointsNumber = int.Parse(strPointsNum);
                Point3D[] points = new Point3D[pointsNumber];

                for (int i = 0; i < pointsNumber; i++)
                {
                    line = sr.ReadLine();
                    string[] point = line.Split(' ');
                    sw.WriteLine(line);
                    double x = double.Parse(point[0], CultureInfo.InvariantCulture.NumberFormat);
                    double y = double.Parse(point[1], CultureInfo.InvariantCulture.NumberFormat);
                    double z = double.Parse(point[2], CultureInfo.InvariantCulture.NumberFormat);
                    points[i] = new Point3D(x, y, z);
                }

                line = sr.ReadLine();
                sw.WriteLine(line);
                line = sr.ReadLine();
                sw.WriteLine(line);
                string strTrianglesNumber = line.Split(' ')[1];
                n = int.Parse(strTrianglesNumber);

                Dictionary<Tuple<int,int>, Edge> borders = new Dictionary<Tuple<int, int>, Edge>();

                for (int i = 0; i < n; i++)
                {
                    int A, B, C;
                    line = sr.ReadLine();
                    sw.WriteLine(line);
                    string[] strNodes = line.Split(' ');
                    A = int.Parse(strNodes[0]);
                    B = int.Parse(strNodes[1]);
                    C = int.Parse(strNodes[2]);
                    AddTriangleEdge(borders, A, B, points, i);
                    AddTriangleEdge(borders, B, C, points, i);
                    AddTriangleEdge(borders, A, C, points, i);
                }

                distances = new int[n];
                nodesList = new List<List<Edge>>(n);

                for (int i = 0; i < n; i++)
                {
                    distances[i] = -1;
                    nodesList.Add(new List<Edge>());
                }

                foreach (Edge edge in borders.Values)
                {
                    if (edge.To == -1)
                        continue;
                    nodesList[edge.From].Add(edge);
                    nodesList[edge.To].Add(edge.GetReversed());
                }
            }
        }

        private void AddTriangleEdge(Dictionary<Tuple<int, int>, Edge> borders,
            int A, int B, Point3D[] points, int curTriangleNumber)
        {
            Tuple<int, int> border = new Tuple<int, int>(Math.Min(A, B), Math.Max(A, B));
            Edge edge;
            if (borders.TryGetValue(border, out edge))
            {
                edge.To = curTriangleNumber;
            }
            else
            {
                edge = new Edge(Point3D.Distance(points[A], points[B]), curTriangleNumber, -1);
                borders.Add(border, edge);
            }
        }

        public void InitializeGraphFromTestFile(string filename)
        {
            using (StreamReader sr = new StreamReader(new FileStream(filename, FileMode.Open)))
            {
                string nmString = sr.ReadLine();
                string[] nmArray = nmString.Split(' ');
                n = int.Parse(nmArray[0]);
                m = int.Parse(nmArray[1]);

                distances = new int[n];
                nodesList = new List<List<Edge>>(n);

                for (int i = 0; i < n; i++)
                {
                    nodesList.Add(new List<Edge>());
                }

                for (int i = 0; i < m; i++)
                {
                    string[] strValues = sr.ReadLine().Split(' ');
                    int firstNode = int.Parse(strValues[0]);
                    int secondNode = int.Parse(strValues[1]);
                    double weight = double.Parse(strValues[2], CultureInfo.InvariantCulture.NumberFormat);
                    Edge edge = new Edge(weight, firstNode, secondNode);
                    nodesList[firstNode].Add(edge);
                    nodesList[secondNode].Add(edge.GetReversed());
                }
            }
        }

        public struct Point3D
        {
            public double X { get; private set; }
            public double Y { get; private set; }
            public double Z { get; private set; }

            public Point3D(double x, double y, double z)
            {
                X = x;
                Y = y;
                Z = z;
            }

            public static double Distance(Point3D first, Point3D second)
            {
                return Math.Sqrt(Math.Pow(first.X - second.X, 2)
                                 + Math.Pow(first.Y - second.Y, 2)
                                 + Math.Pow(first.Z - second.Z, 2));
            }
        }
        public sealed class Edge
        {
            public int From { get; set; }
            public int To { get; set; }
            public double Weight { get; private set; }

            public Edge(double weight, int from, int to)
            {
                this.Weight = weight;
                this.From = from;
                this.To = to;
            }

            public override string ToString()
            {
                return $"{nameof(From)}: {From}, {nameof(To)}: {To}, {nameof(Weight)}: {Weight}";
            }

            public Edge GetReversed()
            {
                return new Edge(Weight, To, From);
            }
        }

        public sealed class FlowEdge
        {
            public int From { get; set; }
            public int To { get; set; }
            public double Cap { get; set; } = 42;
            public double Flow { get; set; }

            public FlowEdge(int from, int to, double cap, double flow)
            {
                From = from;
                To = to;
                Cap = cap;
                Flow = flow;
            }
        }
    }
}
