#Puts the object into the environment
#or updates the object if already added
byte ADD=0

#Removes the object from the environment entirely
byte REMOVE=1

#Only valid within the context of a CollisionAttachedObject message
#Will be ignored if sent with an CollisionObject message
#Takes an attached object, detaches from the attached link
#But adds back in as regular object
byte DETACH_AND_ADD_AS_OBJECT=2

#Only valid within the context of a CollisionAttachedObject message
#Will be ignored if sent with an CollisionObject message
#Takes current object in the environment and removes it as
#a regular object
byte ATTACH_AND_REMOVE_AS_OBJECT=3

# Byte code for operation
byte operation
