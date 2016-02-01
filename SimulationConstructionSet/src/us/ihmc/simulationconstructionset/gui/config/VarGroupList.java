package us.ihmc.simulationconstructionset.gui.config;

//~--- JDK imports ------------------------------------------------------------

import java.util.ArrayList;

public class VarGroupList {
    private ArrayList<VarGroup> groups = new ArrayList<VarGroup>();

    public VarGroupList() {}

    public void addVarGroup(VarGroup group) {
        groups.add(group);
    }

    public String[] getVarGroupNames() {
        int      n   = groups.size();
        String[] ret = new String[n];

        for (int i = 0; i < n; i++) {
            ret[i] = (groups.get(i)).getName();
        }

        return ret;
    }

    public VarGroup getVarGroup(String name) {
        int n = groups.size();

        for (int i = 0; i < n; i++) {
            VarGroup group = (groups.get(i));

            if (group.getName().equals(name)) {
                return group;
            }
        }

        return null;
    }

    public void setupVarGroup(String name, String[] vars) {
        VarGroup group = new VarGroup(name);

        group.addVars(vars);
        addVarGroup(group);
    }

    public void setupVarGroup(String name, String[] vars, String[] regularExpressions) {
        VarGroup group = new VarGroup(name);

        group.addVars(vars);
        group.addRegularExpressions(regularExpressions);
        addVarGroup(group);
    }

    /*
     * public void setupVarGroup(String name, String regex)
     * {
     * VarGroup group = new VarGroup(name);
     * group.addRegularExpression(regex);
     *
     * addVarGroup(group);
     * }
     */
}
