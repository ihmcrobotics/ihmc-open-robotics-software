/*
 * Copyright (c) 2015, Tal Regev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Android Sensors Driver nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package us.ihmc.perception;

import java.util.HashMap;
import java.util.Map;

//from http://stackoverflow.com/questions/11047756/getting-enum-associated-with-int-value
public enum OpenCVColorFormats
{
   INVALID(-1), GRAY(0), RGB(1), BGR(2), RGBA(3), BGRA(4), YUV422(5), BAYER_RGGB(6), BAYER_BGGR(7), BAYER_GBRG(8), BAYER_GRBG(9);
   protected int encodingNumber;

   private static final Map<Integer, OpenCVColorFormats> map = new HashMap<>();

   static
   {
      for (OpenCVColorFormats encoding : OpenCVColorFormats.values())
      {
         map.put(encoding.encodingNumber, encoding);
      }
   }

   OpenCVColorFormats(final int encodingNumber)
   {
      this.encodingNumber = encodingNumber;
   }

   public static OpenCVColorFormats valueOf(int encodingNumber)
   {
      return map.get(encodingNumber);
   }
}
