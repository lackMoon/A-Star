package bearmaps.hw4;

import edu.princeton.cs.algs4.Stopwatch;

import java.util.*;

public class AStarSolver<Vertex> implements ShortestPathsSolver<Vertex> {

    private SolverOutcome outcome;
    private double solutionWeight;
    private List<Vertex> solution;
    private double timeSpent;

    private int numStatesExplored;

    public class Item {
        Item(Vertex v, double priority) {
            this.v = v;
            this.priority = priority;
        }
        Vertex v;

        double priority;

    }

    public AStarSolver(AStarGraph<Vertex> input, Vertex start, Vertex end, double timeout) {
        PriorityQueue<Item> pq = new PriorityQueue<>((v1, v2) -> (int) (v1.priority - v2.priority));
        Map<Vertex, Double> distTo = new HashMap<>();
        Map<Vertex, Vertex> edgeTo = new HashMap<>();
        solution = new ArrayList<>();
        Stopwatch sw = new Stopwatch();
        distTo.put(start, 0.0);
        edgeTo.put(start, null);
        pq.add(new Item(start , 0.0 + input.estimatedDistanceToGoal(start, end)));
        while (!pq.isEmpty()) {
            timeSpent = sw.elapsedTime();
            if (timeSpent >= timeout) {
                outcome = SolverOutcome.TIMEOUT;
                return;
            }
            Item item = pq.remove();
            Vertex vertex = item.v;
            if (item.priority != distTo.get(vertex) + input.estimatedDistanceToGoal(vertex, end)) {
                continue;
            }
            numStatesExplored++;
            if (vertex.equals(end)) {
                outcome = SolverOutcome.SOLVED;
                solutionWeight = distTo.get(end);
                Vertex solutionVertex = end;
                while (solutionVertex != null) {
                    solution.add(solutionVertex);
                    solutionVertex = edgeTo.get(solutionVertex);
                }
                Collections.reverse(solution);
                return;
            }
            List<WeightedEdge<Vertex>> neighbors = input.neighbors(vertex);
            for(WeightedEdge<Vertex> edge : neighbors) {
                Vertex source = edge.from();
                Vertex dest = edge.to();
                double distance = distTo.get(source) + edge.weight();
                if (!distTo.containsKey(dest) || distance < distTo.get(dest)) {
                    distTo.put(dest, distance);
                    edgeTo.put(dest, source);
                    pq.add(new Item(dest, distance + input.estimatedDistanceToGoal(dest, end)));
                }
            }
        }
        outcome = SolverOutcome.UNSOLVABLE;
        timeSpent = sw.elapsedTime();
    }
    public SolverOutcome outcome() {
        return outcome;
    }
    public List<Vertex> solution() {
        return solution;
    }
    public double solutionWeight() {
        return outcome == SolverOutcome.SOLVED ? solutionWeight : 0;
    }
    public int numStatesExplored() {
        return numStatesExplored;
    }
    public double explorationTime() {
        return timeSpent;
    }
}
