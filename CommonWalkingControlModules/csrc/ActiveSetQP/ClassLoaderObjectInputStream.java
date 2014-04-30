import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectStreamClass;

class ClassLoaderObjectInputStream extends ObjectInputStream
{
        public ClassLoaderObjectInputStream(InputStream in) throws IOException
        {
            super(in);
        }
        protected Class resolveClass(ObjectStreamClass desc)
                throws IOException, ClassNotFoundException
        {
            try
            {
                return super.resolveClass(desc);
            } catch (ClassNotFoundException e) {
                return Class.forName(desc.getName());
            }
        }
}

