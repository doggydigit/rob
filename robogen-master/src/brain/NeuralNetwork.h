/*
 * @(#) NeuralNetwork.h   1.0   March 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_NEURAL_NETWORK_H_
#define ROBOGEN_NEURAL_NETWORK_H_

#define MAX_INPUT_NEURONS 13
#define MAX_OUTPUT_NEURONS 6

/*
 * No namespace here on purpose ;-)
 */

typedef struct {

	/*
	 * Given m input neurons and n output neurons
	 * m <= MAX_INPUT_NEURONS
	 * n <= MAX_OUTPUT_NEURONS
	 *
	 * One weight for each input-output connection (w_ij, input neuron i, 0 <= i <= m, output neuron j, 0 <= j <= n
	 * One weight for each output-output connection (wr_ij, output neuron i,j, 0 <= i,j <= n )
	 *
	 * The weights are saved as the concatenation by row of the following:
	 * w_00 w_01 ... w_0n
	 * w_10 w_11 ... w_1n
	 * ...  ...  ... ...
	 * w_m0 w_m1 ... w_mn
	 *
	 * wo_00 wo_01 ... wo_0n
	 * wo_10 wo_11 ... wo_1n
	 * ...  ...  ... ....
	 * wo_n0 wo_n1 ... wo_nn
	 */
	float weight[MAX_INPUT_NEURONS * MAX_OUTPUT_NEURONS
			+ MAX_OUTPUT_NEURONS * MAX_OUTPUT_NEURONS];

	/*
	 * One bias for each output neuron
	 */
	float bias[MAX_OUTPUT_NEURONS];

	/*
	 * One gain for each output neuron
	 */
	float gain[MAX_OUTPUT_NEURONS];

	/*
	 * One state for each output neuron
	 * The state has double the space to store also the next
	 * value.
	 */
	float state[MAX_OUTPUT_NEURONS*2];

	/**
	 * Indicates at which index of the state array the current state starts
	 * (alternatively curStateStart = 0 or curStateStart = n/2)
	 */
	int curStateStart;

	/**
	 * Onje input state for each input neuron
	 */
	float input[MAX_INPUT_NEURONS];

	/**
	 * The number of inputs
	 */
	unsigned int nInputs;

	/**
	 * The number of outputs
	 */
	unsigned int nOutputs;

} NeuralNetwork;

/**
 * Initializes a NeuralNetwork data structure
 * @param network the neural network
 * @param nInputs the number of inputs of the neural network
 * @param nOutputs the number of outputs of the neural network
 * @param weight weights of the neural network. Weights must be provided in the same order as
 *               specified in the NeuralNetwork structure
 * @param bias the bias of each output neuron
 * @param gain the gain of each output neuron
 */
void initNetwork(NeuralNetwork* network, unsigned int nInputs, unsigned int nOutputs,
		const float *weights, const float* bias, const float* gain);

/**
 * Feed the neural network with input values
 * @param network the neural network
 * @param input the input values, must be an array of m inputs
 */
void feed(NeuralNetwork* network, const float *input);

/**
 * Step the neural network of 1 timestep
 * @param network the neural network
 */
void step(NeuralNetwork* network);

/**
 * Read the output of the neural network
 * @param network the neural network
 * @param output the output of the neural network, must point to an area of memory of at least size n
 */
void fetch(const NeuralNetwork* network, float *output);


#endif /* ROBOGEN_NEURAL_NETWORK_H_ */
