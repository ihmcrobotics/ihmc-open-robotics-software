# This message describes the feedback gain computed by the solver.
#
# A state-feedback gain is used to apply state feedback control.
# Generally speaking, this is a matrix defined by the dimension of state and control spaces.
# Both dimensions need to described in nx and nu, respectively.
#
# The values of this gain are stored in data array.
# The entries of the matrix are stored in row-major order.
# The entire first row is stored first, followed by the entire second row, and so on.

# This represents the dimension of the state and control
uint32 nx # number of columns
uint32 nu # number of rows

# This represents the entries of the matrix allocated by row order.
float64[] data