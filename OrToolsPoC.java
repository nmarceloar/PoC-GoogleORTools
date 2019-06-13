package example;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.google.ortools.constraintsolver.Assignment;
import com.google.ortools.constraintsolver.FirstSolutionStrategy;
import com.google.ortools.constraintsolver.RoutingDimension;
import com.google.ortools.constraintsolver.RoutingIndexManager;
import com.google.ortools.constraintsolver.RoutingModel;
import com.google.ortools.constraintsolver.RoutingSearchParameters;
import com.google.ortools.constraintsolver.main;

class UnfeasibleProblemException extends RuntimeException {}

public class PoC {

    static {

	System.loadLibrary("jniortools");

    }

    static class Dimensions {

	final static String TotalTime = "TotalTime";

    }

    static class DataModel {

	public final String[] sites =
		{ "DEPOT", "Bombonera", "Luna Park", "Obelisco", "Casa Rosada" };

	public final long[][] distanceMatrix = {
		{ 0, 0, 0, 0, 0 },
		{ 0, 0, 17, 21, 19 },
		{ 0, 13, 0, 12, 10 },
		{ 0, 19, 9, 0, 13 },
		{ 0, 17, 11, 16, 0 } };

	public final long[][] sitesTimeWindows = {
		{ 0 * 60, 0 * 60 },
		{ 9 * 60, 18 * 60 },
		{ 9 * 60, 18 * 60 },
		{ 9 * 60, 18 * 60 },
		{ 9 * 60, 18 * 60 } };

	public final int siteCount = sites.length;

	public final int userCount = 1;

	public long[][] userTimeWindow = { { 11 * 60, 17 * 60 } };

	public final int[] start = { 0 };

	public final int[] end = { 0 };

	public long[] stayingTime = { 0, 35, 45, 65, 50 };

    }

    static Stream<Integer> range(int from, int to) {

	return Stream.iterate(from, i -> i + 1)
		.limit(to);

    }

    public static void main(String[] args) throws Exception {

	final DataModel data = new DataModel();

	RoutingIndexManager manager =
		new RoutingIndexManager(data.siteCount, data.userCount, data.start, data.end);

	RoutingModel routing = new RoutingModel(manager);

	final int transitCallbackIndex = routing.registerTransitCallback((fromIndex, toIndex) -> {

	    return data.stayingTime[manager.indexToNode(fromIndex)]
		    + data.distanceMatrix[manager.indexToNode(fromIndex)][manager
			    .indexToNode(toIndex)];

	});

	routing.setArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

	routing.addDimension(transitCallbackIndex,
		Long.MAX_VALUE,
		Long.MAX_VALUE,
		false,
		Dimensions.TotalTime);

	RoutingDimension totalTime = routing.getMutableDimension(Dimensions.TotalTime);

	range(0, data.siteCount).skip(1)
		.forEach(i -> {

		    totalTime.cumulVar(manager.nodeToIndex(i))
			    .setRange(data.sitesTimeWindows[i][0],
				    data.sitesTimeWindows[i][1] - data.stayingTime[i]);

		});

	range(0, data.userCount).forEach(i -> {

	    totalTime.cumulVar(routing.start(i))
		    .setMin(data.userTimeWindow[i][0]);

	    totalTime.cumulVar(routing.end(i))
		    .setMax(data.userTimeWindow[i][1]);

	});

	RoutingSearchParameters searchParameters = main.defaultRoutingSearchParameters()
		.toBuilder()
		.setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
		.build();

	PoC.printSolution(PoC.buildSolution(data,
		routing,
		manager,
		totalTime,
		Optional.ofNullable(routing.solveWithParameters(searchParameters))
			.orElseThrow(UnfeasibleProblemException::new)));

    }

    private static void printSolution(final List<List<?>> solution) {

	System.out.println(String.format("Start: %s",
		solution.get(0)
			.get(1)));

	solution.stream()
		.skip(1)
		.limit(solution.size() - 2)
		.forEach(System.out::println);

	System.out.println(String.format("End: %s",
		solution.get(solution.size() - 1)
			.get(1)));

    }

    static List<List<?>> buildSolution(DataModel data,
	    RoutingModel routing,
	    RoutingIndexManager manager,
	    RoutingDimension timeDimension,
	    Assignment solution) {

	return range(0, data.userCount)
		.flatMap(i -> Stream
			.iterate(routing.start(i), index -> routing.next(solution, index))
			.limit(data.siteCount + 1))
		.map((index) -> Arrays.asList(data.sites[manager.indexToNode(index)],
			time(solution.min(timeDimension.cumulVar(index))),
			time(solution.max(timeDimension.cumulVar(index)))))
		.collect(Collectors.toList());

    }

    static String time(long m) {

	return String.format("%02d:%02d hs", m / 60, m % 60);

    }

}
