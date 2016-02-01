package us.ihmc.simulationconstructionset.util.neuralnetwork;


public class CMAC
{
   private int C;    // = 20;
   private int TABLE_SIZE;    // = 1500;  /* 701,4096 */
   private int DIMENSION;    // = 8;
   private double BETA;    // = 0.4;

   private int[] excited;
   private double[] weight;

   private int[] xmax;
   private int[][] number_of_sensors;    // [C][DIMENSION],
   private int[] number_of_gates;    // [C],
   private int[] base_index;    // [C],
   private int[][] p;    // [C][DIMENSION],
   private int[] sensor_excited;    // [DIMENSION];

   private double[] MAX_VALS, MIN_VALS;

   public static void main(String[] args)
   {
      int DIM = 4;
      CMAC cmac = new CMAC(4, 20, 197700, 0.4, new double[] {0.0, 0.0, 0.0, 0.0}, new double[] {1.0, 1.0, 1.0, 1.0});
      double[] x = new double[DIM];

      for (int i = 0; i < 1000000; i++)
      {
         for (int j = 0; j < DIM; j++)
         {
            x[j] = Math.random();
         }

         double y = testFunction(x);
         cmac.trainNet(x, y);

      }

      x[1] = 0.3;
      x[2] = 0.25;
      x[3] = 0.7;

      for (double x0 = 0.0; x0 < 1.0; x0 = x0 + 0.01)
      {
         x[0] = x0;
         System.out.println("x = (" + x[0] + ", " + x[1] + ", " + x[2] + ", " + x[3] + ")  CMAC: " + cmac.calculateOutput(x) + "  ; Real: " + testFunction(x));

      }

   }

   private static double testFunction(double[] x)
   {
      return (x[0] * x[1] + Math.sin(x[2] * Math.PI) + Math.cos(x[3] * Math.PI));
   }

   // DIMENSION is the number of variables in x, C is the generalization factor, TABLE_SIZE is the size of the hash table,
   // BETA is the learning parameter
   public CMAC(int DIMENSION, int C, int TABLE_SIZE, double BETA, double[] MIN_VALS, double[] MAX_VALS)
   {
      this.C = C;
      this.TABLE_SIZE = TABLE_SIZE;
      this.DIMENSION = DIMENSION;
      this.BETA = BETA;

      this.MIN_VALS = MIN_VALS;
      this.MAX_VALS = MAX_VALS;

      excited = new int[C];
      weight = new double[TABLE_SIZE];

      xmax = new int[DIMENSION];    // {128,128,128,128,128,128,128,128}

      for (int i = 0; i < DIMENSION; i++)
      {
         xmax[i] = 128;
      }

      number_of_sensors = new int[C][DIMENSION];
      number_of_gates = new int[C];
      base_index = new int[C];

      p = new int[C][DIMENSION];
      sensor_excited = new int[DIMENSION];

      x_as_int = new int[DIMENSION];

      init_cmac();
   }

   private void convertDoubleToInt(double[] x_in, int[] x_out)
   {
      for (int i = 0; i < DIMENSION; i++)
      {
         x_out[i] = (int) (xmax[i] * (x_in[i] - MIN_VALS[i]) / (MAX_VALS[i] - MIN_VALS[i]));

         // System.out.println(x_out[i]);
      }

   }

   private int[] x_as_int;

   public double calculateOutput(double[] x_in)
   {
      convertDoubleToInt(x_in, x_as_int);

      return calculateOutput(x_as_int);
   }

   private double calculateOutput(int[] x_in)
   {
      double output;

      for (int l = 0; l < C; l++)
      {
         excited[l] = hash(number_of_the_excited_andgate(l, x_in));
      }

      output = 0.0;

      for (int ii = 0; ii < C; ii++)
      {
         output += weight[excited[ii]];
      }

      return (output);
   }


   public void trainNet(double[] x_in, double y)
   {
      convertDoubleToInt(x_in, x_as_int);
      trainNet(x_as_int, y);
   }

   private void trainNet(int[] x_in, double y)
   {
      double output, error;

      output = calculateOutput(x_in);
      error = y - output;


      for (int j = 0; j < C; j++)
      {
         weight[excited[j]] += BETA / C * error;
      }

      // System.out.println("y: " + y + ", output: " + output + ", error: " + error + ", new_output: " + calculateOutput(x_in));
   }

   private void init_cmac()
   {
      int[] nrf = new int[DIMENSION];

      base_index[0] = 0;

      for (int i = 0; i < C; i++)
      {
         number_of_gates[i] = 1;

         for (int k = 0; k < DIMENSION; k++)
         {
            nrf[k] = number_of_receptive_fields(i, k);
            number_of_sensors[i][k] = nrf[k];
            number_of_gates[i] *= nrf[k];
         }

         // printf("layer %2d has been initialized with %3d and-gates \n",
//       i,number_of_gates[i]);
         if (i < C - 1)
            base_index[i + 1] = base_index[i] + number_of_gates[i];
      }

      for (int i = 0; i < C; i++)
      {
         for (int k = 0; k < DIMENSION; k++)
         {
            if (k == 0)
               p[i][k] = 1;
            else
               p[i][k] = number_of_sensors[i][k] * p[i][k - 1];
         }
      }
   }



   private int number_of_the_excited_andgate(int layer, int[] x)
   {
      int l = layer;
      int offset = layer;
      int pp[] = new int[DIMENSION];
      int k;

      for (k = 0; k < DIMENSION; k++)
      {
         sensor_excited[k] = (x[k] + offset) / C;
         pp[k] = p[l][k];
      }

      return (dot_product(sensor_excited, pp) + base_index[l]);
   }

   private int hash(int andgate_number)
   {
      return (andgate_number % TABLE_SIZE);
   }


   private int number_of_receptive_fields(int layer, int direction)
   {
      int xi = xmax[direction] + layer + 1;

      if ((xi % C) == 0)
         return xi / C;
      else
         return xi / C + 1;
   }

   private int dot_product(int[] a, int[] b)
   {
      int temp = 0;

      for (int k = 0; k < DIMENSION; k++)
      {
         temp += a[k] * b[k];
      }

      return temp;
   }

}
