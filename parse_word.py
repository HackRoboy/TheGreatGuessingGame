#!/usr/bin/env python3

def ratings(word, start=0):
  # got_results = True
  # while got_results:
  import subprocess
  from xml.etree.ElementTree import ElementTree
  import lxml.html
  
  def get_links(i):
      res = []
      for a in i:
          if a.tag == 'a':
              res.append(a.text_content())
          for b in a:
              if b.tag == 'a':
                  res.append(b.text_content())
              for c in b:
                  if c.tag == 'a':
                      res.append(c.text_content())
                  for d in c:
                      if d.tag == 'a':
                          res.append(d.text_content())
      return res
  
  # out = subprocess.check_output(['cat', 'index.html'])
  out = subprocess.check_output(['curl', '-qk', 'https://wordassociations.net/en/words-associated-with/{0}?start={1}'.format(word, start)])
#
  html = lxml.html.fromstring(out)
  res = {}

  def geta(cat):
    res = html.find_class('{0}-SECTION'.format(cat))
    a = get_links(res)
    # print(a)
    return a
  return geta('NOUN'), geta('VERB'), geta('ADJECTIVE')

def increased_ratings(word):
    start = 0
    A, B, C = ratings(word, start=start)
    AA, BB, CC = [], [], []
    AA += A
    BB += B
    CC += C
    while len(A)+len(B)+len(C) != 0:
        start += 100
        A, B, C = ratings(word, start=start)
        AA += A
        BB += B
        CC += C
    return AA, BB, CC

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('words', metavar='WORD', nargs='+',
                    help='an integer for')

    args = parser.parse_args()
    for word in args.words:
        word = word.lower()
        import os
        pathname = 'ratings/{0}.txt'.format(word)
        if os.path.exists(pathname):
            continue
        with open(pathname, 'w') as file:
            A, B, C = increased_ratings(word)
            file.write(' '.join(A))
            file.write("\n")
            file.write(' '.join(B))
            file.write("\n")
            file.write(' '.join(C))
            file.write("\n")
#mce    print(increased_ratings('apple'))
