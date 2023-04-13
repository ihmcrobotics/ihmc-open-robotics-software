// Targeted by JavaCPP version 1.5.8: DO NOT EDIT THIS FILE

package us.ihmc.promp;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import static us.ihmc.promp.global.promp.*;


    /**
     * \brief      Class for handling data from multiple trajectories.
     * Data from each trajectory should be stored in an individual file, 
     * files can be generated from serializer or be in .csv format.
     * Each column within .csv file represents a trajectory.
     * All .csv files must have trajectories(columns) in the same sequence (column-wise)
     */
    @Namespace("promp") @NoOffset @Properties(inherit = us.ihmc.promp.presets.ProMPInfoMapper.class)
public class TrajectoryGroup extends Pointer {
        static { Loader.load(); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public TrajectoryGroup(Pointer p) { super(p); }
        /** Native array allocator. Access with {@link Pointer#position(long)}. */
        public TrajectoryGroup(long size) { super((Pointer)null); allocateArray(size); }
        private native void allocateArray(long size);
        @Override public TrajectoryGroup position(long position) {
            return (TrajectoryGroup)super.position(position);
        }
        @Override public TrajectoryGroup getPointer(long i) {
            return new TrajectoryGroup((Pointer)this).offsetAddress(i);
        }
    
        
        /**
         *  \brief default constructor: create an empty trajectory group
         */
        public TrajectoryGroup() { super((Pointer)null); allocate(); }
        private native void allocate();

        /**
         *  \brief    load trajectories from list of files (formatted as generated from io/serializer). 
         *	\param	files of files.
         *	\param	index list of indexes representing dofs to keep.
         */
        public native void load_trajectories(@Const @ByRef StringVector files, @Const @ByRef SizeTVector index);

        /**
         *  \brief    load trajectories from list of .csv files. 
         *	\param	files of files.
         *	\param	index list of indexes representing dofs to keep.
         *	\param	sep values separator.
         *	\param	skip_header if true skip first line.
         */
        public native void load_csv_trajectories(@Const @ByRef StringVector files,
                    @Const @ByRef SizeTVector index, @Cast("char") byte sep/*=','*/, @Cast("bool") boolean skip_header/*=false*/);
        public native void load_csv_trajectories(@Const @ByRef StringVector files,
                    @Const @ByRef SizeTVector index);

        /**
         *  \brief    load trajectories from list of .csv files. 
         *	\param	files of files.
         *	\param	cols list of columns (dofs) to keep.
         *	\param	sep values separator.
         */
        public native void load_csv_trajectories(@Const @ByRef StringVector files,
                    @Const @ByRef StringVector cols, @Cast("char") byte sep/*=','*/);
        public native void load_csv_trajectories(@Const @ByRef StringVector files,
                    @Const @ByRef StringVector cols);

        /**
         *   \brief    Normalize all trajectories to the mean length (number of timesteps)
         */
        public native long normalize_length();

        /**
         *  \brief     Normalize all trajectories to the same desired length
         *	\param	len desired length
         */
        

        /**
         *   \brief    standardize each dof among the trajectories
         */
        public native @ByVal VectorVectorPair standardize_dims();

        /**
         * \brief	return the vector of trajectories
         */
        public native @Const @ByRef TrajectoryVector trajectories();

        /**
         * \brief	return the mean end value of the trajectory
         * @param dof of the trajectory of interest
         */
        public native double get_mean_end_value(int dof);

        public native double get_mean_start_value(int dof);
    }
