package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Genetic algorithm used to tune PID gains. Each chromosome contains: [kP, kI, kD]
 */
public class PIDGeneticAlgorithm {

    private final Random random = new Random();
    private final int populationSize;
    private final int generations;
    private final double mutationRate;
    private final double crossoverRate;
    private final double[] lowerBounds;
    private final double[] upperBounds;
    private final FitnessFunction fitnessFunction;

    public PIDGeneticAlgorithm(
            int populationSize,
            int generations,
            double mutationRate,
            double crossoverRate,
            double[] lowerBounds,
            double[] upperBounds,
            FitnessFunction fitnessFunction
    ) {
        this.populationSize = populationSize;
        this.generations = generations;
        this.mutationRate = mutationRate;
        this.crossoverRate = crossoverRate;
        this.lowerBounds = lowerBounds.clone();
        this.upperBounds = upperBounds.clone();
        this.fitnessFunction = fitnessFunction;
    }

    /**
     * Runs the genetic algorithm and returns the best PID values.
     *
     * @return {bestKP, bestKI, bestKD}
     */
    public double[] tune() {
        List<double[]> population = createInitialPopulation();

        double[] bestGenes = population.get(0).clone();
        double bestFitness = fitnessFunction.evaluate(
                bestGenes[0],
                bestGenes[1],
                bestGenes[2]
        );

        for (int generation = 0; generation < generations; generation++) {
            List<Individual> rankedPopulation = rankPopulation(population);

            if (rankedPopulation.get(0).fitness > bestFitness) {
                bestFitness = rankedPopulation.get(0).fitness;
                bestGenes = rankedPopulation.get(0).genes.clone();
            }

            List<double[]> nextGeneration = new ArrayList<>();

            // Keep the best PID values unchanged.
            nextGeneration.add(bestGenes.clone());

            while (nextGeneration.size() < populationSize) {
                double[] parentA = tournamentSelect(rankedPopulation);
                double[] parentB = tournamentSelect(rankedPopulation);

                double[] child;

                if (random.nextDouble() < crossoverRate) {
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

        return bestGenes;
    }

    private List<double[]> createInitialPopulation() {
        List<double[]> population = new ArrayList<>();

        for (int i = 0; i < populationSize; i++) {
            double[] genes = new double[3];

            genes[0] = randomInRange(lowerBounds[0], upperBounds[0]);
            genes[1] = randomInRange(lowerBounds[1], upperBounds[1]);
            genes[2] = randomInRange(lowerBounds[2], upperBounds[2]);

            population.add(genes);
        }

        return population;
    }

    private List<Individual> rankPopulation(List<double[]> population) {
        List<Individual> rankedPopulation = new ArrayList<>();

        for (double[] genes : population) {
            double fitness = fitnessFunction.evaluate(
                    genes[0],
                    genes[1],
                    genes[2]
            );

            rankedPopulation.add(new Individual(genes, fitness));
        }

        rankedPopulation.sort(
                (a, b) -> Double.compare(b.fitness, a.fitness)
        );

        return rankedPopulation;
    }

    private double[] tournamentSelect(List<Individual> rankedPopulation) {
        int tournamentSize = 3;

        double[] bestGenes = null;
        double bestFitness = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < tournamentSize; i++) {
            Individual candidate = rankedPopulation.get(
                    random.nextInt(rankedPopulation.size())
            );

            if (candidate.fitness > bestFitness) {
                bestFitness = candidate.fitness;
                bestGenes = candidate.genes;
            }
        }

        return bestGenes.clone();
    }

    private double[] crossover(double[] parentA, double[] parentB) {
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

    private void mutate(double[] genes) {
        for (int i = 0; i < 3; i++) {
            if (random.nextDouble() < mutationRate) {
                double range = upperBounds[i] - lowerBounds[i];
                double mutationSize = range * 0.15;

                genes[i] += random.nextGaussian() * mutationSize;
            }
        }
    }

    private void clampToBounds(double[] genes) {
        for (int i = 0; i < 3; i++) {
            genes[i] = Math.max(
                    lowerBounds[i],
                    Math.min(upperBounds[i], genes[i])
            );
        }
    }

    private double randomInRange(double min, double max) {
        return min + random.nextDouble() * (max - min);
    }

    public interface FitnessFunction {
        double evaluate(double kP, double kI, double kD);
    }

    private static class Individual {
        final double[] genes;
        final double fitness;

        Individual(double[] genes, double fitness) {
            this.genes = genes;
            this.fitness = fitness;
        }
    }
}
