void kernel text_parameter_gets_passed_in(__global int* index)
{
   int gid = get_global_id(0);
   printf("index[%d] = %d\n", gid, index[gid]);
}