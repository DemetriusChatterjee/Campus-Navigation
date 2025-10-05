import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Scanner;
import java.util.Set;
/*
 * Author: Demetrius Chatterjee
 * Class: CampusNavigation
 * Description: This class is used to read the graph from the file and store it in an adjacency list.
 *              It provides methods for DFS, BFS traversals and finding the minimum spanning tree
 *              of the campus paths.
 * Github ID: DemetriusChatterjee
 * Version: 1.0
 */
public class CampusNavigation {
    // Map to store adjacency list representation
    private Map<String, Map<String, Integer>> graph;
    
    // Add these constants for validation
    private static final int MAX_WALKING_TIME = 30; // maximum reasonable walking time in minutes
    
    // Add this inner class for edge representation
    /*
     * Author: Demetrius Chatterjee
     * Class: Edge
     * Description: This class represents an edge in the graph, containing the source building,
     *              destination building, and the walking time between them.
     * Github ID: DemetriusChatterjee
     * Version: 1.0
     */
    private static class Edge {
        String from;
        String to;
        int weight;
        
        Edge(String from, String to, int weight) {
            this.from = from;
            this.to = to;
            this.weight = weight;
        }
    }
    
    // Add this class for Union-Find data structure
    /*
     * Author: Demetrius Chatterjee
     * Class: UnionFind
     * Description: This class implements the Union-Find data structure used in Kruskal's algorithm
     *              to detect cycles when building the minimum spanning tree.
     * Github ID: DemetriusChatterjee
     * Version: 1.0
     */
    private static class UnionFind {
        private Map<String, String> parent;
        private Map<String, Integer> rank;
        
        UnionFind(Set<String> vertices) {
            parent = new HashMap<>();
            rank = new HashMap<>();
            for (String vertex : vertices) {
                parent.put(vertex, vertex);
                rank.put(vertex, 0);
            }
        }
        
        /**
         * Finds the root (representative) of a vertex in the Union-Find structure.
         * Uses path compression for efficiency.
         * @param vertex The vertex to find the root for
         * @return The root vertex
         */
        String find(String vertex) {
            if (!parent.get(vertex).equals(vertex)) {
                parent.put(vertex, find(parent.get(vertex)));
            }
            return parent.get(vertex);
        }
        
        /**
         * Unites two sets in the Union-Find structure.
         * Uses union by rank for efficiency.
         * @param x First vertex
         * @param y Second vertex
         */
        void union(String x, String y) {
            String rootX = find(x);
            String rootY = find(y);
            
            if (rank.get(rootX) < rank.get(rootY)) {
                parent.put(rootX, rootY);
            } else if (rank.get(rootX) > rank.get(rootY)) {
                parent.put(rootY, rootX);
            } else {
                parent.put(rootY, rootX);
                rank.put(rootX, rank.get(rootX) + 1);
            }
        }
    }
    
    /*
     * Constructor: CampusNavigation
     * Description: This constructor is used to initialize the graph.
     * Author: Demetrius Chatterjee
     * Github ID: DemetriusChatterjee
     * Version: 1.0
     */
    public CampusNavigation() {
        graph = new HashMap<>();
    }
    
    /*
     * Method: readFile
     * Description: This method is used to read the graph from the file and store it in an adjacency list.
     * Author: Demetrius Chatterjee
     * Github ID: DemetriusChatterjee
     * Version: 1.0
     */
    /**
     * Reads stock data from a CSV file and populates the tree.
     * @param filename The path to the CSV file
     * @throws IOException if there's an error reading the file
     * @throws IllegalArgumentException if the file format or data is invalid
     */
    public void readFile(String filename) throws IOException {
        Set<String> declaredNodes = new HashSet<>();
        
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            boolean readingNodes = false;
            boolean readingEdges = false;
            
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) continue;
                
                if (line.equals("NODES")) {
                    readingNodes = true;
                    readingEdges = false;
                    continue;
                } else if (line.equals("EDGES")) {
                    readingNodes = false;
                    readingEdges = true;
                    continue;
                }
                
                if (readingNodes) {
                    if (declaredNodes.contains(line)) {
                        throw new IllegalArgumentException("Duplicate node: " + line);
                    }
                    declaredNodes.add(line);
                    graph.put(line, new HashMap<>());
                }
                
                if (readingEdges) {
                    String[] parts = line.split(",");
                    if (parts.length != 3) {
                        throw new IllegalArgumentException("Invalid edge format: " + line);
                    }
                    
                    String from = parts[0].trim();
                    String to = parts[1].trim();
                    int distance;
                    
                    try {
                        distance = Integer.parseInt(parts[2].trim());
                    } catch (NumberFormatException e) {
                        throw new IllegalArgumentException("Invalid distance format: " + parts[2]);
                    }
                    
                    // Validate nodes exist
                    if (!declaredNodes.contains(from)) {
                        throw new IllegalArgumentException("Undefined building in edge: " + from);
                    }
                    if (!declaredNodes.contains(to)) {
                        throw new IllegalArgumentException("Undefined building in edge: " + to);
                    }
                    
                    // Validate distance
                    if (distance <= 0) {
                        throw new IllegalArgumentException("Distance must be positive: " + distance);
                    }
                    if (distance > MAX_WALKING_TIME) {
                        throw new IllegalArgumentException("Unreasonable walking time: " + distance);
                    }
                    
                    graph.get(from).put(to, distance);
                    graph.get(to).put(from, distance);
                }
            }
        }
    }
    
    /*
     * Method: dfs
     * Description: Performs a depth-first search traversal starting from the given building
     */
    /**
     * Performs a depth-first search traversal starting from the given building.
     * @param startBuilding The building to start the traversal from
     * @return List of buildings in DFS traversal order
     * @throws IllegalArgumentException if the starting building doesn't exist
     */
    public List<String> dfs(String startBuilding) {
        if (!graph.containsKey(startBuilding)) {
            throw new IllegalArgumentException("Starting building does not exist");
        }
        
        List<String> visited = new ArrayList<>();
        Set<String> visitedSet = new HashSet<>();
        dfsHelper(startBuilding, visitedSet, visited);
        return visited;
    }
    
    /**
     * Helper method for DFS traversal.
     * @param current The current building being visited
     * @param visitedSet Set of buildings already visited
     * @param visited List to store the traversal order
     */
    private void dfsHelper(String current, Set<String> visitedSet, List<String> visited) {
        visitedSet.add(current);
        visited.add(current);
        
        for (String neighbor : graph.get(current).keySet()) {
            if (!visitedSet.contains(neighbor)) {
                dfsHelper(neighbor, visitedSet, visited);
            }
        }
    }
    
    /*
     * Method: bfs
     * Description: Performs a breadth-first search traversal starting from the given building
     */
    /**
     * Performs a breadth-first search traversal starting from the given building.
     * @param startBuilding The building to start the traversal from
     * @return List of buildings in BFS traversal order
     * @throws IllegalArgumentException if the starting building doesn't exist
     */
    public List<String> bfs(String startBuilding) {
        if (!graph.containsKey(startBuilding)) {
            throw new IllegalArgumentException("Starting building does not exist");
        }
        
        List<String> visited = new ArrayList<>();
        Set<String> visitedSet = new HashSet<>();
        Queue<String> queue = new LinkedList<>();
        
        queue.offer(startBuilding);
        visitedSet.add(startBuilding);
        
        while (!queue.isEmpty()) {
            String current = queue.poll();
            visited.add(current);
            
            for (String neighbor : graph.get(current).keySet()) {
                if (!visitedSet.contains(neighbor)) {
                    visitedSet.add(neighbor);
                    queue.offer(neighbor);
                }
            }
        }
        
        return visited;
    }
    
    /**
     * Prints the traversal path in a formatted way with arrows between buildings.
     * @param path List of buildings in traversal order
     */
    private void printTraversal(List<String> path) {
        System.out.println(String.join(" -> ", path));
    }
    
    /*
     * Method: run
     * Description: Runs the main menu system
     */
    /**
     * Main method to run the campus navigation program.
     * Loads the graph from a file and provides an interactive menu for various operations.
     * @param filename The input file containing the campus map data
     */
    public void run(String filename) {
        Scanner scanner = new Scanner(System.in);
        
        try {
            readFile(filename);
            System.out.println("\nWelcome to Campus Navigator!");
            System.out.printf("Loaded campus map with %d buildings and %d paths.\n", 
                graph.size(), countEdges());
            
            while (true) {
                System.out.println("\nChoose algorithm:");
                System.out.println("1. Depth First Search");
                System.out.println("2. Breadth First Search");
                System.out.println("3. Minimum Spanning Tree");
                System.out.println("4. Find Shortest Path");
                System.out.println("5. Exit");
                
                System.out.print("\nEnter choice: ");
                String choice = scanner.nextLine().trim();
                
                if (choice.equals("5")) {
                    break;
                }
                
                if (choice.equals("4")) {
                    System.out.print("Enter start building: ");
                    String start = scanner.nextLine().trim();
                    System.out.print("Enter end building: ");
                    String end = scanner.nextLine().trim();
                    printShortestPathDirections(start, end);
                    continue;
                }
                
                if (choice.equals("3")) {
                    List<Edge> mst = findMinimumSpanningTree();
                    printMST(mst);
                    continue;
                }
                
                if (!choice.equals("1") && !choice.equals("2")) {
                    System.out.println("Invalid choice. Please try again.");
                    continue;
                }
                
                System.out.print("Enter starting building: ");
                String startBuilding = scanner.nextLine().trim();
                
                if (!graph.containsKey(startBuilding)) {
                    System.out.println("Error: Building does not exist!");
                    continue;
                }
                
                List<String> traversal;
                if (choice.equals("1")) {
                    System.out.println("\nDFS Traversal from " + startBuilding + ":");
                    traversal = dfs(startBuilding);
                } else {
                    System.out.println("\nBFS Traversal from " + startBuilding + ":");
                    traversal = bfs(startBuilding);
                }
                
                printTraversal(traversal);
            }
            
        } catch (IOException e) {
            System.err.println("Error loading file: " + e.getMessage());
        } finally {
            scanner.close();
        }
    }
    
    /*
     * Method: countEdges
     * Description: Counts the total number of unique edges in the graph
     */
    /**
     * Counts the total number of unique edges in the graph.
     * Since this is an undirected graph, each edge is counted only once.
     * @return The number of unique edges
     */
    private int countEdges() {
        int count = 0;
        for (Map<String, Integer> edges : graph.values()) {
            count += edges.size();
        }
        // Divide by 2 since each edge is counted twice in an undirected graph
        return count / 2;
    }
    
    // Add Kruskal's algorithm implementation
    /**
     * Finds the minimum spanning tree of the campus using Kruskal's algorithm.
     * @return List of edges that form the minimum spanning tree
     */
    public List<Edge> findMinimumSpanningTree() {
        // Create priority queue for edges
        PriorityQueue<Edge> edges = new PriorityQueue<>(
            Comparator.comparingInt(e -> e.weight)
        );
        
        // Add all edges to priority queue
        Set<String> addedEdges = new HashSet<>();
        for (String from : graph.keySet()) {
            for (Map.Entry<String, Integer> entry : graph.get(from).entrySet()) {
                String to = entry.getKey();
                String edgeKey = from.compareTo(to) < 0 ? 
                    from + "," + to : to + "," + from;
                
                if (!addedEdges.contains(edgeKey)) {
                    edges.offer(new Edge(from, to, entry.getValue()));
                    addedEdges.add(edgeKey);
                }
            }
        }
        
        // Initialize Union-Find data structure
        UnionFind uf = new UnionFind(graph.keySet());
        List<Edge> mst = new ArrayList<>();
        
        // Process edges in order of increasing weight
        while (!edges.isEmpty() && mst.size() < graph.size() - 1) {
            Edge edge = edges.poll();
            if (!uf.find(edge.from).equals(uf.find(edge.to))) {
                mst.add(edge);
                uf.union(edge.from, edge.to);
            }
        }
        
        return mst;
    }
    
    /**
     * Prints the minimum spanning tree results including total walking time.
     * @param mst List of edges in the minimum spanning tree
     */
    private void printMST(List<Edge> mst) {
        System.out.println("\nMinimum Spanning Tree Results:");
        int totalWeight = 0;
        for (Edge edge : mst) {
            // Compare building names to determine display order
            String from = edge.from;
            String to = edge.to;
            if (from.compareTo(to) > 0) {
                // Swap if 'from' comes after 'to' alphabetically
                String temp = from;
                from = to;
                to = temp;
            }
            
            System.out.printf("%s -> %s (%d minute%s)\n", 
                from, to, edge.weight,
                edge.weight == 1 ? "" : "s");
            totalWeight += edge.weight;
        }
        System.out.printf("\nTotal walking time: %d minutes\n", totalWeight);
    }
    
    /**
     * Class to store distance and path information for Dijkstra's algorithm
     */
    private static class DijkstraInfo {
        int distance;
        List<String> path;
        
        DijkstraInfo(int distance, List<String> path) {
            this.distance = distance;
            this.path = path;
        }
    }
    
    /**
     * Finds the shortest path between two buildings using Dijkstra's algorithm.
     * @param start The starting building
     * @param end The destination building
     * @return DijkstraInfo containing the shortest distance and path
     * @throws IllegalArgumentException if either building doesn't exist
     */
    public DijkstraInfo findShortestPath(String start, String end) {
        if (!graph.containsKey(start)) {
            throw new IllegalArgumentException("Start building does not exist: " + start);
        }
        if (!graph.containsKey(end)) {
            throw new IllegalArgumentException("End building does not exist: " + end);
        }
        
        // Priority queue to store vertices with their distances
        PriorityQueue<Map.Entry<String, Integer>> pq = 
            new PriorityQueue<>(Comparator.comparingInt(Map.Entry::getValue)); //makes the priority queue a min heap
        
        // Store distances and previous nodes for path reconstruction
        Map<String, Integer> distances = new HashMap<>();
        Map<String, String> previous = new HashMap<>();
        
        // Initialize distances
        for (String building : graph.keySet()) {
            distances.put(building, Integer.MAX_VALUE);
        }
        distances.put(start, 0);
        pq.offer(new AbstractMap.SimpleEntry<>(start, 0));
        
        // Dijkstra's algorithm
        while (!pq.isEmpty()) {
            Map.Entry<String, Integer> current = pq.poll();
            String currentBuilding = current.getKey();
            int currentDistance = current.getValue();
            
            if (currentBuilding.equals(end)) {
                break;
            }
            
            if (currentDistance > distances.get(currentBuilding)) {
                continue;
            }
            
            for (Map.Entry<String, Integer> neighbor : graph.get(currentBuilding).entrySet()) { //changed from entrySet to entrySet()
                String nextBuilding = neighbor.getKey();
                int newDistance = currentDistance + neighbor.getValue();
                
                if (newDistance < distances.get(nextBuilding)) { //if the new distance is less than the current distance, update the distance and previous
                    distances.put(nextBuilding, newDistance);
                    previous.put(nextBuilding, currentBuilding);
                    pq.offer(new AbstractMap.SimpleEntry<>(nextBuilding, newDistance));
                }
            }
        }
        
        // Reconstruct path
        List<String> path = new ArrayList<>();
        String current = end;
        
        while (current != null) {
            path.add(0, current);
            current = previous.get(current);
        }
        
        // Check if path exists
        if (path.size() <= 1 || !path.get(0).equals(start)) {
            throw new IllegalStateException("No path exists between " + start + " and " + end);
        }
        
        return new DijkstraInfo(distances.get(end), path);
    }
    
    /**
     * Prints detailed directions for the shortest path between two buildings.
     * @param start The starting building
     * @param end The destination building
     */
    public void printShortestPathDirections(String start, String end) {
        try {
            DijkstraInfo result = findShortestPath(start, end);
            
            System.out.println("\nShortest Path Analysis:");
            System.out.println("Path: " + String.join(" -> ", result.path));
            System.out.printf("Total time: %d minutes\n", result.distance);
            
            System.out.println("\nDetailed directions:");
            System.out.println("1. Start at " + start);
            
            for (int i = 0; i < result.path.size() - 1; i++) {
                String current = result.path.get(i);
                String next = result.path.get(i + 1);
                int time = graph.get(current).get(next);
                System.out.printf("%d. Walk to %s (%d minute%s)\n", 
                    i + 2, next, time, time == 1 ? "" : "s");
            }
            
        } catch (IllegalArgumentException | IllegalStateException e) {
            System.out.println("Error: " + e.getMessage());
        }
    }
    
    /*
     * Method: main
     * Description: Entry point of the program
     */
    /**
     * Entry point of the program.
     * @param args Command line arguments - expects a single argument for the input file path
     */
    public static void main(String[] args) {
        CampusNavigation navigator = new CampusNavigation();
        boolean test = false;            //CHANGE THIS TO FALSE TO RUN THE PROGRAM
        if (test) {
            navigator.run("campus_map.txt");
        }else{
            if (args.length != 1) {
                System.out.println("Usage: java CampusNavigation <input-file>");
                return;
            }
            navigator.run(args[0]);
        }
    }
}
