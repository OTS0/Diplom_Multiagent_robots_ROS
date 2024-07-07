def find_neighbors(index, width, height, costmap, orthogonal_step_cost):
  """
  Identifies neighbor nodes inspecting the 8 adjacent neighbors
  Checks if neighbor is inside the map boundaries and if is not an obstacle according to a threshold
  Returns a list with valid neighbour nodes as [index, step_cost] pairs
  """
  neighbors = []
  # length of diagonal = length of one side by the square root of 2 (1.41421)
  diagonal_step_cost = orthogonal_step_cost * 1.41421 
  # threshold value used to reject neighbor nodes as they are considered as obstacles [1-254]
  lethal_cost = 90
  gab_side =16
  upper = index - width
  upper_gab = index - width*gab_side
  if upper > 0 and upper_gab>0:
    if costmap[upper] < lethal_cost and costmap[upper_gab] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[upper]/255
      neighbors.append([upper, step_cost])

  left = index - 1
  left_gab = index - gab_side-1
  if left % width > 0 and left_gab % width > 0 :
    if costmap[left] < lethal_cost and costmap[left_gab] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[left]/255
      neighbors.append([left, step_cost])

  upper_left = index - width - 1
  upper_left_gab = index - width*gab_side - 1-gab_side
  if upper_left > 0 and upper_left % width > 0 and upper_left_gab % width > 0:
    if costmap[upper_left] < lethal_cost and costmap[upper_left_gab] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_left]/255
      neighbors.append([index - width - 1, step_cost])

  upper_right = index - width + 1
  upper_right_gab = index - width*gab_side + 1+gab_side
  if upper_right > 0 and (upper_right) % width != (width - 1) and upper_right_gab > 0 and (upper_right_gab) % width != (width - 1):
    if costmap[upper_right] < lethal_cost and costmap[upper_right_gab] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_right]/255
      neighbors.append([upper_right, step_cost])

  right = index + 1
  right_gab = index + 1+gab_side
  if right % width != (width + 1) and right_gab % width != (width + 1):
    if costmap[right] < lethal_cost and costmap[right_gab] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[right]/255
      neighbors.append([right, step_cost])

  lower_left = index + width - 1
  lower_left_gab = index + width*gab_side - 1-gab_side
  if lower_left < height * width and lower_left % width != 0 and lower_left_gab < height * width and lower_left_gab % width != 0:
    if costmap[lower_left] < lethal_cost and costmap[lower_left_gab] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_left]/255
      neighbors.append([lower_left, step_cost])

  lower = index + width
  lower_gab = index + width*gab_side
  if lower <= height * width and lower_gab <= height * width:
    if costmap[lower] < lethal_cost and costmap[lower_gab] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[lower]/255
      neighbors.append([lower, step_cost])


  lower_right = index + width + 1
  lower_right_gab = index + width*gab_side + 1+gab_side

  if (lower_right) <= height * width and lower_right % width != (width - 1) and (lower_right_gab) <= height * width and lower_right_gab % width != (width - 1):
    if costmap[lower_right] < lethal_cost and costmap[lower_right_gab] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_right]/255
      neighbors.append([lower_right, step_cost])

  return neighbors

def find_weighted_neighbors(index, width, height, costmap, orthogonal_step_cost):
  """
  Identifies neighbor nodes inspecting the 8 adjacent neighbors
  Checks if neighbor is inside the map boundaries and if is not an obstacle according to a threshold
  Returns a list with valid neighbour nodes as [index, step_cost] pairs
  """
  neighbors = []
  # length of diagonal = length of one side by the square root of 2 (1.41421)
  diagonal_step_cost = orthogonal_step_cost * 1.41421
  # threshold value used to reject neighbor nodes as they are considered as obstacles [1-254]
  lethal_cost = 150
  step_cost = 0

  upper = index - width
  if upper > 0:
    if costmap[upper] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[upper]/255
    else:
      step_cost = float('inf')
    neighbors.append([upper, step_cost])

  left = index - 1
  if left % width > 0:
    if costmap[left] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[left]/255
    else:
      step_cost = float('inf')
    neighbors.append([left, step_cost])

  upper_left = index - width - 1
  if upper_left > 0 and upper_left % width > 0:
    if costmap[upper_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_left]/255
    else:
      step_cost = float('inf')  
    neighbors.append([index - width - 1, step_cost])

  upper_right = index - width + 1
  if upper_right > 0 and (upper_right) % width != (width - 1):
    if costmap[upper_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_right]/255
    else:
      step_cost = float('inf')  
    neighbors.append([upper_right, step_cost])

  right = index + 1
  if right % width != (width + 1):
    if costmap[right] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[right]/255
    else:
      step_cost = float('inf')
    neighbors.append([right, step_cost])

  lower_left = index + width - 1
  if lower_left < height * width and lower_left % width != 0:
    if costmap[lower_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_left]/255
    else:
      step_cost = float('inf')
    neighbors.append([lower_left, step_cost])

  lower = index + width
  if lower <= height * width:
    if costmap[lower] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[lower]/255
    else:
      step_cost = float('inf')
    neighbors.append([lower, step_cost])

  lower_right = index + width + 1
  if (lower_right) <= height * width and lower_right % width != (width - 1):
    if costmap[lower_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_right]/255
    else:
      step_cost = float('inf')
    neighbors.append([lower_right, step_cost])

  return neighbors