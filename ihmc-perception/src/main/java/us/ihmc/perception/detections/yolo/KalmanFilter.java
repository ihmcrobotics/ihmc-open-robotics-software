package us.ihmc.perception.detections.yolo;

/**
 *
 * State (8D): [x, y, w, h, vx, vy, vw, vh]
 * Meas  (4D): [x, y, w, h]
 *
 * x,y are center coordinates.
 *
 * Noise scaling matches the reference:
 *   std_weight_position = 1/20
 *   std_weight_velocity = 1/160
 *
 * initiate():
 *   std = [
 *     2*pos*w, 2*pos*h, 2*pos*w, 2*pos*h,
 *     10*vel*w, 10*vel*h, 10*vel*w, 10*vel*h
 *   ]
 *
 * predict():
 *   motion_cov diag uses:
 *     [pos*w, pos*h, pos*w, pos*h, vel*w, vel*h, vel*w, vel*h]^2
 *
 * project():
 *   innovation_cov diag uses:
 *     [pos*w, pos*h, pos*w, pos*h]^2
 *
 * update():
 *   uses Cholesky solve implicitly (here: we implement a stable-ish 4x4 inverse with pivoting).
 *   If you have EJML available, use it instead for S^-1 / solves.
 */

public class KalmanFilter
{
   private static final int dimX = 8;
   private static final int dimZ = 4;

   private static final float STD_WEIGHT_POSITION = 1.0f / 20.0f;
   private static final float STD_WEIGHT_VELOCITY = 1.0f / 160.0f;

   private final float[][] F = new float[dimX][dimX];
   private final float[][] H = new float[dimZ][dimX];

   private final float[] mean = new float[dimX];
   private final float[][] cov = new float[dimX][dimX];

   public KalmanFilter()
   {
      // F: constant velocity, dt=1
      for (int i = 0; i < dimX; i++)
         F[i][i] = 1f;
      for (int i = 0; i < 4; i++)
         F[i][i + 4] = 1f;

      // H: observe [x, y, w, h]
      for (int i = 0; i < dimZ; i++)
         H[i][i] = 1f;

      // initialize cov to identity-ish
      for (int i = 0; i < dimX; i++)
         cov[i][i] = 1f;
   }

   /** measurement z = [x, y, w, h] */
   public void initiate(float[] z)
   {
      mean[0] = z[0];
      mean[1] = z[1];
      mean[2] = z[2];
      mean[3] = z[3];
      for (int i = 4; i < dimX; i++)
         mean[i] = 0f;

      float w = Math.max(1f, z[2]);
      float h = Math.max(1f, z[3]);

      float[] std = new float[8];
      std[0] = 2f  * STD_WEIGHT_POSITION * w;
      std[1] = 2f  * STD_WEIGHT_POSITION * h;
      std[2] = 2f  * STD_WEIGHT_POSITION * w;
      std[3] = 2f  * STD_WEIGHT_POSITION * h;
      std[4] = 10f * STD_WEIGHT_VELOCITY * w;
      std[5] = 10f * STD_WEIGHT_VELOCITY * h;
      std[6] = 10f * STD_WEIGHT_VELOCITY * w;
      std[7] = 10f * STD_WEIGHT_VELOCITY * h;

      zero(cov);
      for (int i = 0; i < dimX; i++)
         cov[i][i] = std[i] * std[i];
   }

   public void predict()
   {
      // mean = F * mean
      float[] newMean = mul(F, mean);
      System.arraycopy(newMean, 0, mean, 0, dimX);

      // motion_cov diag: [pos*w, pos*h, pos*w, pos*h, vel*w, vel*h, vel*w, vel*h]^2
      float w = Math.max(1f, mean[2]);
      float h = Math.max(1f, mean[3]);

      float[] stdPos = new float[] {
            STD_WEIGHT_POSITION * w,
            STD_WEIGHT_POSITION * h,
            STD_WEIGHT_POSITION * w,
            STD_WEIGHT_POSITION * h
      };
      float[] stdVel = new float[] {
            STD_WEIGHT_VELOCITY * w,
            STD_WEIGHT_VELOCITY * h,
            STD_WEIGHT_VELOCITY * w,
            STD_WEIGHT_VELOCITY * h
      };

      float[][] Q = new float[dimX][dimX];
      Q[0][0] = stdPos[0] * stdPos[0];
      Q[1][1] = stdPos[1] * stdPos[1];
      Q[2][2] = stdPos[2] * stdPos[2];
      Q[3][3] = stdPos[3] * stdPos[3];
      Q[4][4] = stdVel[0] * stdVel[0];
      Q[5][5] = stdVel[1] * stdVel[1];
      Q[6][6] = stdVel[2] * stdVel[2];
      Q[7][7] = stdVel[3] * stdVel[3];

      float[][] FCFt = mul(mul(F, cov), transpose(F));
      addInPlace(FCFt, Q, cov);
   }

   /** update with measurement z = [x, y, w, h] */
   public void update(float[] z)
   {
      // project
      // projected_mean = H*mean
      float[] projectedMean = mul(H, mean);

      // projected_cov = H*cov*H^T + innovation_cov
      float[][] projectedCov = mul(mul(H, cov), transpose(H));

      float w = Math.max(1f, mean[2]);
      float h = Math.max(1f, mean[3]);

      float[] std = new float[] {
            STD_WEIGHT_POSITION * w,
            STD_WEIGHT_POSITION * h,
            STD_WEIGHT_POSITION * w,
            STD_WEIGHT_POSITION * h
      };

      float[][] innovationCov = new float[dimZ][dimZ];
      innovationCov[0][0] = std[0] * std[0];
      innovationCov[1][1] = std[1] * std[1];
      innovationCov[2][2] = std[2] * std[2];
      innovationCov[3][3] = std[3] * std[3];

      float[][] S = add(projectedCov, innovationCov); // 4x4

      // K = cov*H^T * S^-1
      float[][] covHt = mul(cov, transpose(H)); // 8x4
      float[][] Sinv = inv4(S);
      float[][] K = mul(covHt, Sinv); // 8x4

      // innovation = z - projectedMean
      float[] y = subVec(z, projectedMean);

      // mean = mean + K*y
      float[] Ky = mul(K, y);
      for (int i = 0; i < dimX; i++)
         mean[i] += Ky[i];

      // cov = cov - K*S*K^T   (matches python: cov - K*projected_cov*K^T ; here S is projected_cov+innov)
      float[][] KSKt = mul(mul(K, S), transpose(K)); // 8x8
      float[][] newCov = sub(cov, KSKt);
      copy(newCov, cov);
   }

   public float[] getMean()
   {
      float[] out = new float[dimX];
      System.arraycopy(mean, 0, out, 0, dimX);
      return out;
   }

   // ---------------- helpers ----------------

   private static float[] subVec(float[] a, float[] b)
   {
      float[] r = new float[a.length];
      for (int i = 0; i < a.length; i++) r[i] = a[i] - b[i];
      return r;
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

   private static void zero(float[][] A)
   {
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            A[i][j] = 0f;
   }

   private static void copy(float[][] src, float[][] dst)
   {
      for (int i = 0; i < src.length; i++)
         System.arraycopy(src[i], 0, dst[i], 0, src[i].length);
   }

   private static void addInPlace(float[][] A, float[][] B, float[][] dst)
   {
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            dst[i][j] = A[i][j] + B[i][j];
   }

   // 4x4 inverse with partial pivoting (ok for KF; EJML would be better)
   private static float[][] inv4(float[][] A)
   {
      int n = 4;
      float[][] aug = new float[n][2*n];
      for (int i = 0; i < n; i++)
      {
         for (int j = 0; j < n; j++) aug[i][j] = A[i][j];
         aug[i][n+i] = 1f;
      }

      for (int col = 0; col < n; col++)
      {
         int pivotRow = col;
         float best = Math.abs(aug[col][col]);
         for (int r = col + 1; r < n; r++)
         {
            float v = Math.abs(aug[r][col]);
            if (v > best) { best = v; pivotRow = r; }
         }
         if (pivotRow != col)
         {
            float[] tmp = aug[col];
            aug[col] = aug[pivotRow];
            aug[pivotRow] = tmp;
         }

         float pivot = aug[col][col];
         if (Math.abs(pivot) < 1e-9f)
            pivot = (pivot >= 0f ? 1e-9f : -1e-9f);

         for (int j = 0; j < 2*n; j++) aug[col][j] /= pivot;

         for (int r = 0; r < n; r++)
         {
            if (r == col) continue;
            float f = aug[r][col];
            if (f == 0f) continue;
            for (int j = 0; j < 2*n; j++) aug[r][j] -= f * aug[col][j];
         }
      }

      float[][] inv = new float[n][n];
      for (int i = 0; i < n; i++)
         System.arraycopy(aug[i], n, inv[i], 0, n);
      return inv;
   }
}