package us.ihmc.perception;

import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.H5File;

import java.util.HashMap;

public class HDF5Manager {

    private HashMap<String, Group> groups;
    private H5File file;

    public HDF5Manager(String filePath, int flag) {
        file = new H5File(filePath, flag);
        groups = new HashMap<>();
    }

    public Group getGroup(String namespace) {
        if (groups.containsKey(namespace)) {
            return groups.get(namespace);
        } else {
            Group group = file.createGroup(namespace);
            groups.put(namespace, group);
            return group;
        }
    }

    public long getCount(String namespace) {
        if (groups.containsKey(namespace)) return groups.get(namespace).getNumObjs();
        else return 0;
    }

    public H5File getFile() {
        return file;
    }

    public HashMap<String, Group> getGroups() {
        return groups;
    }
}
