class DotDict(dict):
  """dot.notation access to dictionary attributes"""
  __getattr__ = dict.get
  __setattr__ = dict.__setitem__
  __delattr__ = dict.__delitem__

  # convert back to normal dict 
  def to_dict(self):
    dict = {}
    for key, value in self.items():
        dict[key] = value
    return dict