import os
os.system("make.bat rinoh")
os.system("make.bat singlehtml")
os.chdir('build/singlehtml')

import bs4   #BeautifulSoup 3 has been replaced
soup = bs4.BeautifulSoup(open("index.html").read(),features="html.parser")
stylesheets = soup.findAll("link", {"rel": "stylesheet"})
for s in stylesheets:
    t = soup.new_tag('style')
    c = bs4.element.NavigableString(open(s["href"]).read())
    t.insert(0,c)
    t['type'] = 'text/css'
    s.replaceWith(t)
open("../../../docs/index.html", "w").write(str(soup))




