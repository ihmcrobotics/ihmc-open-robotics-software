package us.ihmc.robotDataLogger.util;

import org.openjdk.jol.info.ClassLayout;

/**
 * 
 * Padded volatile reference that is guaranteed to be on its own 64 bit cache line
 * 
 * Compressed object references take 4 bytes. This structure forces the desired object to be in its own cacheline
 * 
 * @author Jesper Smith
 *
 * @param <T>
 */
public class PaddedVolatileReference<T>
{
   private volatile Object o0;
   private volatile Object o1;
   private volatile Object o2;
   private volatile Object o3;
   private volatile Object o4;
   private volatile Object o5;
   private volatile Object o6;
   private volatile Object o7;
   private volatile Object o8;
   private volatile Object o9;
   private volatile Object o10;
   private volatile Object o11;
   private volatile Object o12;
   private volatile Object o13;
   private volatile Object o14;

   private volatile Object paddedVolatile;
   private volatile Object o15;
   
   private volatile Object o16;
   private volatile Object o17;
   private volatile Object o18;
   private volatile Object o19;
   private volatile Object o20;
   private volatile Object o21;
   private volatile Object o22;
   private volatile Object o23;
   private volatile Object o24;
   private volatile Object o25;
   private volatile Object o26;
   private volatile Object o27;
   private volatile Object o28;
   private volatile Object o29;

   
   public PaddedVolatileReference()
   {
      
   }
   
   public PaddedVolatileReference(T initial)
   {
      set(initial);
   }
   
   
   @SuppressWarnings("unchecked")
   public T get()
   {
      return (T) paddedVolatile;
   }
   
   
   public void set(T obj)
   {
      paddedVolatile = obj;
   }
   
   
   @Deprecated public Object getO0()
   {
      return o0;
   }

   @Deprecated public Object getO1()
   {
      return o1;
   }

   @Deprecated public Object getO2()
   {
      return o2;
   }

   @Deprecated public Object getO3()
   {
      return o3;
   }

   @Deprecated public Object getO4()
   {
      return o4;
   }

   @Deprecated public Object getO5()
   {
      return o5;
   }

   @Deprecated public Object getO6()
   {
      return o6;
   }

   @Deprecated public Object getO8()
   {
      return o8;
   }

   @Deprecated public Object getO9()
   {
      return o9;
   }

   @Deprecated public Object getO10()
   {
      return o10;
   }

   @Deprecated public Object getO11()
   {
      return o11;
   }

   @Deprecated public Object getO12()
   {
      return o12;
   }

   @Deprecated public Object getO13()
   {
      return o13;
   }

   @Deprecated public Object getO14()
   {
      return o14;
   }


   @Deprecated public void setO0(Object o0)
   {
      this.o0 = o0;
   }

   @Deprecated public void setO1(Object o1)
   {
      this.o1 = o1;
   }

   @Deprecated public void setO2(Object o2)
   {
      this.o2 = o2;
   }

   @Deprecated public void setO3(Object o3)
   {
      this.o3 = o3;
   }

   @Deprecated public void setO4(Object o4)
   {
      this.o4 = o4;
   }

   @Deprecated public void setO5(Object o5)
   {
      this.o5 = o5;
   }

   @Deprecated public void setO6(Object o6)
   {
      this.o6 = o6;
   }

   @Deprecated public void setO8(Object o8)
   {
      this.o8 = o8;
   }

   @Deprecated public void setO9(Object o9)
   {
      this.o9 = o9;
   }

   @Deprecated public void setO10(Object o10)
   {
      this.o10 = o10;
   }

   @Deprecated public void setO11(Object o11)
   {
      this.o11 = o11;
   }

   @Deprecated public void setO12(Object o12)
   {
      this.o12 = o12;
   }

   @Deprecated public void setO13(Object o13)
   {
      this.o13 = o13;
   }

   @Deprecated public void setO14(Object o14)
   {
      this.o14 = o14;
   }
   
   
   
   @Deprecated public Object getO7()
   {
      return o7;
   }


   @Deprecated public void setO7(Object o7)
   {
      this.o7 = o7;
   }


   @Deprecated public Object getO15()
   {
      return o15;
   }


   @Deprecated public void setO15(Object o15)
   {
      this.o15 = o15;
   }


   @Deprecated public Object getO16()
   {
      return o16;
   }


   @Deprecated public void setO16(Object o16)
   {
      this.o16 = o16;
   }


   @Deprecated public Object getO17()
   {
      return o17;
   }


   @Deprecated public void setO17(Object o17)
   {
      this.o17 = o17;
   }


   @Deprecated public Object getO18()
   {
      return o18;
   }


   @Deprecated public void setO18(Object o18)
   {
      this.o18 = o18;
   }


   @Deprecated public Object getO19()
   {
      return o19;
   }


   @Deprecated public void setO19(Object o19)
   {
      this.o19 = o19;
   }


   @Deprecated public Object getO20()
   {
      return o20;
   }


   @Deprecated public void setO20(Object o20)
   {
      this.o20 = o20;
   }


   @Deprecated public Object getO21()
   {
      return o21;
   }


   @Deprecated public void setO21(Object o21)
   {
      this.o21 = o21;
   }


   @Deprecated public Object getO22()
   {
      return o22;
   }


   @Deprecated public void setO22(Object o22)
   {
      this.o22 = o22;
   }


   @Deprecated public Object getO23()
   {
      return o23;
   }


   @Deprecated public void setO23(Object o23)
   {
      this.o23 = o23;
   }


   @Deprecated public Object getO24()
   {
      return o24;
   }


   @Deprecated public void setO24(Object o24)
   {
      this.o24 = o24;
   }


   @Deprecated public Object getO25()
   {
      return o25;
   }


   @Deprecated public void setO25(Object o25)
   {
      this.o25 = o25;
   }


   @Deprecated public Object getO26()
   {
      return o26;
   }


   @Deprecated public void setO26(Object o26)
   {
      this.o26 = o26;
   }


   @Deprecated public Object getO27()
   {
      return o27;
   }


   @Deprecated public void setO27(Object o27)
   {
      this.o27 = o27;
   }


   @Deprecated public Object getO28()
   {
      return o28;
   }


   @Deprecated public void setO28(Object o28)
   {
      this.o28 = o28;
   }


   @Deprecated
   public Object getO29()
   {
      return o29;
   }


   @Deprecated public void setO29(Object o29)
   {
      this.o29 = o29;
   }


   public static void main(String[] args)
   {
      System.out.println(ClassLayout.parseClass(PaddedVolatileReference.class).toPrintable());
   }


}
