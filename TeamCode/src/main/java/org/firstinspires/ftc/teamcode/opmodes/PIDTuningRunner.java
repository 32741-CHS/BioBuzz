package org.firstinspires.ftc.teamcode.opmodes;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Scanner;

public class PIDTuningRunner {

    private static final Random random = new Random();

    private static final int POPULATION_SIZE = 10;
    private static final int GENERATIONS = 5;
    private static final double MUTATION_RATE = 0.20;
    private static final double CROSSOVER_RATE = 0.80;

    private static final double[] LOWER_BOUNDS = {
            0.005, 0.000, 0.000
    };

    private static final double[] UPPER_BOUNDS = {
            0.080, 0.002, 0.050
    };

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        List<double[]> population = createInitialPopulation();

        double[] bestGenes = population.get(0).clone();
        double bestFitness = Double.NEGATIVE_INFINITY;

        for (int generation = 0; generation < GENERATIONS; generation++) {
            System.out.println();
            System.out.println("Generation " + (generation + 1));
            System.out.println("Test each PID set on the robot.");
            System.out.println("Enter the fitness score from the Driver Station.");

            List<double[]> nextGeneration = new ArrayList<>();

            for (int i = 0; i < POPULATION_SIZE; i++) {
                double[] genes = population.get(i);

                System.out.println();
                System.out.println("Candidate " + (i + 1));
                System.out.println("kP = " + genes[0]);
                System.out.println("kI = " + genes[1]);
                System.out.println("kD = " + genes[2]);

                System.out.print("Enter fitness score: ");
                double fitness = scanner.nextDouble();

                if (fitness > bestFitness) {
                    bestFitness = fitness;
                    bestGenes = genes.clone();
                }
            }

            nextGeneration.add(bestGenes.clone());

            while (nextGeneration.size() < POPULATION_SIZE) {
                double[] parentA = selectParent(population);
                double[] parentB = selectParent(population);

                double[] child;

                if (random.nextDouble() < CROSSOVER_RATE) {
                    child = crossover(parentA, parentB);
                } else {
                    child = parentA.clone();
                }

                mutate(child);
                clampToBounds(child);

                nextGeneration.add(child);
            }

            population = nextGeneration;
        }

        System.out.println();
        System.out.println("Best PID values found:");
        System.out.println("kP = " + bestGenes[0]);
        System.out.println("kI = " + bestGenes[1]);
        System.out.println("kD = " + bestGenes[2]);
        System.out.println("Best fitness = " + bestFitness);
    }

    private static List<double[]> createInitialPopulation() {
        List<double[]> population = new ArrayList<>();

        for (int i = 0; i < POPULATION_SIZE; i++) {
            double[] genes = new double[3];

            genes[0] = randomInRange(
                    LOWER_BOUNDS[0],
                    UPPER_BOUNDS[0]
            );

            genes[1] = randomInRange(
                    LOWER_BOUNDS[1],
                    UPPER_BOUNDS[1]
            );

            genes[2] = randomInRange(
                    LOWER_BOUNDS[2],
                    UPPER_BOUNDS[2]
            );

            population.add(genes);
        }

        return population;
    }

    private static double[] selectParent(List<double[]> population) {
        int tournamentSize = 3;

        double[] bestGenes = null;
        double bestFitness = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < tournamentSize; i++) {
            double[] candidate = population.get(
                    random.nextInt(population.size())
            );

            System.out.print("Enter fitness for candidate: ");
            double fitness = new Scanner(System.in).nextDouble();

            if (fitness > bestFitness) {
                bestFitness = fitness;
                bestGenes = candidate;
            }
        }

        return bestGenes.clone();
    }

    private static double[] crossover(double[] parentA, double[] parentB) {
        double[] child = new double[3];

        for (int i = 0; i < 3; i++) {
            if (random.nextBoolean()) {
                child[i] = parentA[i];
            } else {
                child[i] = parentB[i];
            }
        }

        return child;
    }

    private static void mutate(double[] genes) {
        for (int i = 0; i < 3; i++) {
            if (random.nextDouble() < MUTATION_RATE) {
                double range = UPPER_BOUNDS[i] - LOWER_BOUNDS[i];
                double mutationSize = range * 0.15;

                genes[i] += random.nextGaussian() * mutationSize;
            }
        }
    }

    private static void clampToBounds(double[] genes) {
        for (int i = 0; i < 3; i++) {
            genes[i] = Math.max(
                    LOWER_BOUNDS[i],
                    Math.min(UPPER_BOUNDS[i], genes[i])
            );
        }
    }

    private static double randomInRange(double min, double max) {
        return min + random.nextDouble() * (max - min);
    }
}

