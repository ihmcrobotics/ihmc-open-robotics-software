package us.ihmc.perception.detections.yolo;

public class KalmanFilterXYAH
{
   // State: [cx, cy, a, h, vcx, vcy, va, vh]
   private static final int dimX = 8;
   private static final int dimZ = 4;

   private final float[][] F = new float[dimX][dimX];
   private final float[][] H = new float[dimZ][dimX];

   private float[] mean = new float[dimX];
   private float[][] cov = new float[dimX][dimX];

   // Noise (baseline)
   private final float stdPos = 1.0f;
   private final float stdVel = 0.5f;

   public KalmanFilterXYAH()
   {
      // F: constant velocity, dt=1
      for (int i = 0; i < dimX; i++) F[i][i] = 1f;
      for (int i = 0; i < 4; i++) F[i][i + 4] = 1f;

      // H: observe [cx, cy, a, h]
      for (int i = 0; i < dimZ; i++) H[i][i] = 1f;
   }

   public void initiate(float[] z)
   {
      mean[0] = z[0];
      mean[1] = z[1];
      mean[2] = z[2];
      mean[3] = z[3];
      for (int i = 4; i < dimX; i++) mean[i] = 0f;

      cov = new float[dimX][dimX];
      for (int i = 0; i < 4; i++) cov[i][i] = (stdPos * stdPos) * 10f;
      for (int i = 4; i < 8; i++) cov[i][i] = (stdVel * stdVel) * 100f;
   }

   public void predict()
   {
      mean = mul(F, mean);

      float[][] FCFt = mul(mul(F, cov), transpose(F));

      float[][] Q = new float[dimX][dimX];
      for (int i = 0; i < 4; i++) Q[i][i] = stdPos * stdPos;
      for (int i = 4; i < 8; i++) Q[i][i] = stdVel * stdVel;

      cov = add(FCFt, Q);
   }

   public void update(float[] z)
   {
      float[] y = subVec(z, mul(H, mean)); // innovation

      float[][] R = measurementNoise();
      float[][] S = add(mul(mul(H, cov), transpose(H)), R);

      float[][] K = mul(mul(cov, transpose(H)), inv4(S));

      // mean = mean + K*y
      float[] Ky = mul(K, y);
      for (int i = 0; i < dimX; i++) mean[i] += Ky[i];

      // Joseph form for covariance
      float[][] I = eye(dimX);
      float[][] KH = mul(K, H);
      float[][] IminusKH = sub(I, KH);

      float[][] term1 = mul(mul(IminusKH, cov), transpose(IminusKH));
      float[][] term2 = mul(mul(K, R), transpose(K));
      cov = add(term1, term2);
   }

   public float[] getMean()
   {
      return mean; // full 8D state
   }

   private float[][] measurementNoise()
   {
      float[][] R = new float[dimZ][dimZ];
      for (int i = 0; i < dimZ; i++) R[i][i] = stdPos * stdPos;
      return R;
   }

   // ---- helpers ----
   private static float[] subVec(float[] a, float[] b)
   {
      float[] r = new float[a.length];
      for (int i = 0; i < a.length; i++) r[i] = a[i] - b[i];
      return r;
   }

   private static float[][] eye(int n)
   {
      float[][] I = new float[n][n];
      for (int i = 0; i < n; i++) I[i][i] = 1f;
      return I;
   }

   private static float[][] transpose(float[][] A)
   {
      float[][] T = new float[A[0].length][A.length];
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            T[j][i] = A[i][j];
      return T;
   }

   private static float[][] add(float[][] A, float[][] B)
   {
      float[][] C = new float[A.length][A[0].length];
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            C[i][j] = A[i][j] + B[i][j];
      return C;
   }

   private static float[][] sub(float[][] A, float[][] B)
   {
      float[][] C = new float[A.length][A[0].length];
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            C[i][j] = A[i][j] - B[i][j];
      return C;
   }

   private static float[][] mul(float[][] A, float[][] B)
   {
      int n = A.length, m = B[0].length, k = B.length;
      float[][] C = new float[n][m];
      for (int i = 0; i < n; i++)
         for (int j = 0; j < m; j++)
         {
            float s = 0f;
            for (int t = 0; t < k; t++) s += A[i][t] * B[t][j];
            C[i][j] = s;
         }
      return C;
   }

   private static float[] mul(float[][] A, float[] x)
   {
      float[] y = new float[A.length];
      for (int i = 0; i < A.length; i++)
      {
         float s = 0f;
         for (int j = 0; j < x.length; j++) s += A[i][j] * x[j];
         y[i] = s;
      }
      return y;
   }

   // Same inv4 you had (still fragile but compiles)
   private static float[][] inv4(float[][] A)
   {
      int n = 4;
      float[][] aug = new float[n][2*n];
      for (int i = 0; i < n; i++)
      {
         for (int j = 0; j < n; j++) aug[i][j] = A[i][j];
         aug[i][n+i] = 1f;
      }

      for (int i = 0; i < n; i++)
      {
         float pivot = aug[i][i];
         if (Math.abs(pivot) < 1e-6) pivot = 1e-6f;
         for (int j = 0; j < 2*n; j++) aug[i][j] /= pivot;

         for (int r = 0; r < n; r++)
         {
            if (r == i) continue;
            float f = aug[r][i];
            for (int j = 0; j < 2*n; j++) aug[r][j] -= f * aug[i][j];
         }
      }

      float[][] inv = new float[n][n];
      for (int i = 0; i < n; i++)
         System.arraycopy(aug[i], n, inv[i], 0, n);
      return inv;
   }
}