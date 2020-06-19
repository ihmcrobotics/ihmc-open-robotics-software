import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import java.util.Random;
import org.ejml.dense.row.RandomMatrices_DDRM;

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
		DMatrixRMaj m1,m2;
		DMatrixRMaj r=new DMatrixRMaj(n,n);
		long startTime,startWallTime;



		//ejml
		m1=RandomMatrices_DDRM.rectangle(n,n,new Random(0));
		m2=RandomMatrices_DDRM.rectangle(n,n,new Random(1));
		r= RandomMatrices_DDRM.rectangle(n,n,new Random(2));
		for(int i=0;i<10000;i++)
		{
			CommonOps_DDRM.mult(m1,m2,r);
			//CommonOps_DDRM.insert(r,m2,0,0);
		}
		startTime = System.nanoTime();
		startWallTime = System.currentTimeMillis();
		for(int i=0;i<10000;i++)
		{
			//CommonOps_DDRM.invert(m);
			//CommonOps_DDRM.insert(m, r, 0, 0);
			//CommonOps_DDRM.add(m1,m2,r);
			CommonOps_DDRM.mult(m1,m2,r);
			//CommonOps_DDRM.insert(r,m2,0,0);
		}
		System.out.println("ejml inverse: "+(System.nanoTime()-startTime)*1e-9 + "ms");
		System.out.println("ejml wall : "+(System.currentTimeMillis()-startWallTime)*1e-3 + "ms");
		/*
		System.out.println(m1);
		System.out.println(m2);
		*/
		System.out.println(r);


		//eigen
		m1=RandomMatrices_DDRM.rectangle(n,n,new Random(0));
		m2=RandomMatrices_DDRM.rectangle(n,n,new Random(1));
		r= RandomMatrices_DDRM.rectangle(n,n,new Random(2));
		for(int i=0;i<10000;i++)
		{
			inverse(m1.getData(), m2.getData(),r.getData(), m1.numRows, m1.numCols);
			//CommonOps_DDRM.insert(r,m2,0,0);
		}
		startTime = System.nanoTime();
		startWallTime = System.currentTimeMillis();
		for(int i=0;i<10000;i++)
		{
			
			inverse(m1.getData(), m2.getData(),r.getData(), m1.numRows, m1.numCols);
			//m1=RandomMatrices_DDRM.rectangle(n,n,new Random(0));
			//CommonOps_DDRM.insert(r,m2,0,0);
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
