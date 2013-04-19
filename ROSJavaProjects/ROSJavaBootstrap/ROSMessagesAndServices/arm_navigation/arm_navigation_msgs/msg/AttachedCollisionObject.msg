# The CollisionObject will be attached with a fixed joint to this link
# If link name is set to REMOVE_ALL_ATTACHED_OBJECTS and object.operation 
# is set to REMOVE will remove all attached bodies attached to any object
string link_name

#Reserved for indicating that all attached objects should be removed
string REMOVE_ALL_ATTACHED_OBJECTS = "all"

#This contains the actual shapes and poses for the CollisionObject
#to be attached to the link
#If action is remove and no object.id is set, all objects
#attached to the link indicated by link_name will be removed
CollisionObject object

# The set of links that the attached objects are allowed to touch
# by default - the link_name is included by default
string[] touch_links
