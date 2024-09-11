//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.ros.namespace;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Pattern;
//import org.ros.exception.RosRuntimeException;

public class GraphName {
   @VisibleForTesting
   static final String ANONYMOUS_PREFIX = "anonymous_";
   private static final String ROOT = "/";
   private static final String SEPARATOR = "/";
   private static final Pattern VALID_GRAPH_NAME_PATTERN = Pattern.compile("^([\\~\\/A-Za-z][\\w_\\/]*)?$");
   private static AtomicInteger anonymousCounter = new AtomicInteger();
   private final String name;

   public static GraphName newAnonymous() {
      return new GraphName("anonymous_" + anonymousCounter.incrementAndGet());
   }

   public static GraphName root() {
      return new GraphName("/");
   }

   public static GraphName empty() {
      return new GraphName("");
   }

   public static GraphName of(String name) {
      return new GraphName(canonicalize(name));
   }

   private GraphName(String name) {
      Preconditions.checkNotNull(name);
      this.name = name;
   }

   private static String canonicalize(String name) {
//      if (!VALID_GRAPH_NAME_PATTERN.matcher(name).matches()) {
////         throw new RosRuntimeException("Invalid graph name: " + name);
//      } else {
//         while(!name.equals("/") && name.endsWith("/")) {
//            name = name.substring(0, name.length() - 1);
//         }
//
//         if (name.startsWith("~/")) {
//            name = "~" + name.substring(2);
//         }
//
//         return name;
//      }
      return "";
   }

   public boolean isGlobal() {
      return !this.isEmpty() && this.name.charAt(0) == '/';
   }

   public boolean isRoot() {
      return this.name.equals("/");
   }

   public boolean isEmpty() {
      return this.name.isEmpty();
   }

   public boolean isPrivate() {
      return !this.isEmpty() && this.name.charAt(0) == '~';
   }

   public boolean isRelative() {
      return !this.isPrivate() && !this.isGlobal();
   }

   public GraphName getParent() {
      if (this.name.length() == 0) {
         return empty();
      } else if (this.name.equals("/")) {
         return root();
      } else {
         int slashIdx = this.name.lastIndexOf(47);
         if (slashIdx > 1) {
            return new GraphName(this.name.substring(0, slashIdx));
         } else {
            return this.isGlobal() ? root() : empty();
         }
      }
   }

   public GraphName getBasename() {
      int slashIdx = this.name.lastIndexOf(47);
      if (slashIdx > -1) {
         return slashIdx + 1 < this.name.length() ? new GraphName(this.name.substring(slashIdx + 1)) : empty();
      } else {
         return this;
      }
   }

   public GraphName toRelative() {
      return !this.isPrivate() && !this.isGlobal() ? this : new GraphName(this.name.substring(1));
   }

   public GraphName toGlobal() {
      if (this.isGlobal()) {
         return this;
      } else {
         return this.isPrivate() ? new GraphName("/" + this.name.substring(1)) : new GraphName("/" + this.name);
      }
   }

   public GraphName join(GraphName other) {
      if (!other.isGlobal() && !this.isEmpty()) {
         if (this.isRoot()) {
            return other.toGlobal();
         } else {
            return other.isEmpty() ? this : new GraphName(this.toString() + "/" + other.toString());
         }
      } else {
         return other;
      }
   }

   public GraphName join(String other) {
      return this.join(of(other));
   }

   public String toString() {
      return this.name;
   }

   public int hashCode() {
      return this.name.hashCode();
   }

   public boolean equals(Object obj) {
      if (this == obj) {
         return true;
      } else if (obj == null) {
         return false;
      } else if (this.getClass() != obj.getClass()) {
         return false;
      } else {
         GraphName other = (GraphName)obj;
         if (this.name == null) {
            if (other.name != null) {
               return false;
            }
         } else if (!this.name.equals(other.name)) {
            return false;
         }

         return true;
      }
   }
}
