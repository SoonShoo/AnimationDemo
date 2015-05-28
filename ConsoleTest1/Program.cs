using System;
using System.Text;
using System.Collections.Generic;
using NeuronDotNet.Core.Backpropagation;
using NeuronDotNet.Core;
using btl.generic;
using SkeletonLib;
using BEPUphysics;
using BEPUutilities;

namespace NeuralNetworkTest
{
    public class Program
    {
        public static BackpropagationNetwork network;
        public static Random random;

        public static String weightsToString(double[] weights)
        {
            return String.Join(",", weights);
        }

        public static double[] stringToWeights(String w)
        {
            return Array.ConvertAll(w.Split(','), s => double.Parse(s));
        }

        public static BackpropagationNetwork createNetwork() {
            int x1 = 2 + 18 * 3;
            int x2 = 17 * 3;
            int y = 17 * 3;

            LinearLayer inputLayer = new LinearLayer(x1);
            //SigmoidLayer inputLayer = new SigmoidLayer(x1);
            //SigmoidLayer hiddenLayer = new SigmoidLayer(x2);
            //TanhLayer hiddenLayer = new TanhLayer(x2);
            TanhLayer outputLayer = new TanhLayer(y);

            //BaseSkeleton skeleton = new BaseSkeleton();

            Console.WriteLine("Layers set up");

            BackpropagationConnector connector = new BackpropagationConnector(inputLayer, outputLayer);
            //BackpropagationConnector connector2 = new BackpropagationConnector(hiddenLayer, outputLayer);
            //BackpropagationConnector connector3 = new BackpropagationConnector(hiddenLayer, outputLayer);
            BackpropagationNetwork network = new BackpropagationNetwork(inputLayer, outputLayer);
            network.Initialize();
            return network;
        }

        static void Main(string[] args)
        {
            random = new Random();

            Console.WriteLine("Setting up layers");

            Console.WriteLine("Ga start!!");
            network = createNetwork();
            GA ga = new GA(0.75, 0.1, 90, 300, getWeightsCount(network));
            //GA ga = new GA(0.8, 0.07, 80, 250, getWeightsCount(network));
            ga.FitnessFunction = new GAFunction(fitnessFunction);
            ga.Elitism = true;
            ga.Go();

            double[] weights;
            double fitness;
            ga.GetBest(out weights, out fitness);
            Console.WriteLine("Best brain had a fitness of " + fitness);
            System.IO.StreamWriter file = new System.IO.StreamWriter("d:\\network1.txt");
            file.WriteLine(weightsToString(weights));
            file.Close();
            Console.ReadLine();
        }

        public static int getWeightsCount(BackpropagationNetwork aNetwork)
        {
            int ans = 0;

            foreach (BackpropagationConnector connector in aNetwork.Connectors)
            {
                foreach (BackpropagationSynapse synapse in connector.Synapses)
                {
                    ans += 1;
                    //ans += 1;
                }
            }

            return ans;
        }

        public static void setNetworkWeights(BackpropagationNetwork aNetwork, double[] weights)
        {
            // Setup the network's weights.
            int index = 0;

            foreach (BackpropagationConnector connector in aNetwork.Connectors)
            {
                foreach (BackpropagationSynapse synapse in connector.Synapses)
                {
                    synapse.Weight = weights[index++];
                    //synapse.SourceNeuron.Bias = weights[index++];
                    //index++;
                }
            }
        }

        public static double timeStep = 0.4;
        public static double timeMax = 11;

        public static double fitnessFunction(double[] weights)
        {
            setNetworkWeights(network, weights);
            
            Space space = new Space();
            BaseSkeleton skeleton = new BaseSkeleton(space);
            double time = 0;
            double ans = 0;
            while (time < timeMax) {
                //double[] input = new double[] { time / timeMax, 1};
                double[] input = skeleton.getModelInput(time / timeMax);
                time += timeStep;
                double[] networkOutput = network.Run(input);
                skeleton.setMotorInput(networkOutput);
                space.Update((float)timeStep);
                /*if (time / timeMax > 0.5)
                {
                    ans += (400 - skeleton.getDifferenceBetweenCurrentAndEnd()) / 25;
                }*/
            }
            //return ans;

            return 2000 - skeleton.getDifferenceBetweenCurrentAndEnd();
            
           /* Space space = new Space();
            BaseSkeleton skeleton = new BaseSkeleton(space);
            double[] input = new double[] { 0, 1 };
            double[] networkOutput = network.Run(input);
            double fitness = 50 - skeleton.getDiffMotor(networkOutput);
            return fitness;*/
            /*
            double[] input = new double[] { 1, 1 };
            double temp = 0;
            double[] networkOutput = network.Run(input);
            foreach (double d in networkOutput) {
                temp += Math.Pow(0.8-d, 2);
            }
            return 1000 - temp;*/
        }
    }
}