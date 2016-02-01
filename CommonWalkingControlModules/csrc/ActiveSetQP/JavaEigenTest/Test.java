import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import java.util.Random;
import org.ejml.ops.RandomMatrices;

import com.sun.jna.Native;


class Test
{
	static{
		Native.register("Test");
	}

	public static native void inverse(double[] m1, double[] m2,double[] r, int nRow, int nCol);
	public static void main(String[] arg)
	{

		int n=100;
		DenseMatrix64F m1,m2;
		DenseMatrix64F r=new DenseMatrix64F(n,n);
		long startTime,startWallTime;



		//ejml
		m1=RandomMatrices.createRandom(n,n,new Random(0));
		m2=RandomMatrices.createRandom(n,n,new Random(1));
		r= RandomMatrices.createRandom(n,n,new Random(2));
		for(int i=0;i<10000;i++)
		{
			CommonOps.mult(m1,m2,r);
			//CommonOps.insert(r,m2,0,0);
		}
		startTime = System.nanoTime();
		startWallTime = System.currentTimeMillis();
		for(int i=0;i<10000;i++)
		{
			//CommonOps.invert(m);
			//CommonOps.insert(m, r, 0, 0);
			//CommonOps.add(m1,m2,r);
			CommonOps.mult(m1,m2,r);
			//CommonOps.insert(r,m2,0,0);
		}
		System.out.println("ejml inverse: "+(System.nanoTime()-startTime)*1e-9 + "ms");
		System.out.println("ejml wall : "+(System.currentTimeMillis()-startWallTime)*1e-3 + "ms");
		/*
		System.out.println(m1);
		System.out.println(m2);
		*/
		System.out.println(r);


		//eigen
		m1=RandomMatrices.createRandom(n,n,new Random(0));
		m2=RandomMatrices.createRandom(n,n,new Random(1));
		r= RandomMatrices.createRandom(n,n,new Random(2));
		for(int i=0;i<10000;i++)
		{
			inverse(m1.getData(), m2.getData(),r.getData(), m1.numRows, m1.numCols);
			//CommonOps.insert(r,m2,0,0);
		}
		startTime = System.nanoTime();
		startWallTime = System.currentTimeMillis();
		for(int i=0;i<10000;i++)
		{
			
			inverse(m1.getData(), m2.getData(),r.getData(), m1.numRows, m1.numCols);
			//m1=RandomMatrices.createRandom(n,n,new Random(0));
			//CommonOps.insert(r,m2,0,0);
		}
		System.out.println("eigen inverse: "+(System.nanoTime()-startTime)*1e-9 +"ms");
		System.out.println("eigen wall : "+(System.currentTimeMillis()-startWallTime)*1e-3 + "ms");
		/*
		System.out.println(m1);
		System.out.println(m2);
		*/
		System.out.println(r);

	}
}
