print "native              ",   (0*60 +  4.528)
print "opt0                ",   (9*60 + 20.607)
print "opt2 (inline, noopt)",   (1*60 + 36.334)
print "opt3 (coalesce)     ",   (0*60 + 45.683)
print "opt3 (tls)          ",   (0*60 + 45.371)
print "opt5 (rle+dse)      ",   (0*60 + 21.141)
print "opt6 (avoid aflags) ",   (0*60 + 14.609)
print "opt7 (fold_lea)     ",   (0*60 +  9.821)
print

print "slowdowns from native:"
print "native              ",   (0*60 +  4.528) / (0*60 +  4.528)
print "opt0                ",   (9*60 + 20.607) / (0*60 +  4.528)
print "opt2 (inline, noopt)",   (1*60 + 36.334) / (0*60 +  4.528)
print "opt3 (coalesce)     ",   (0*60 + 45.683) / (0*60 +  4.528)
print "opt3 (tls)          ",   (0*60 + 45.371) / (0*60 +  4.528)
print "opt5 (rle+dse)      ",   (0*60 + 21.141) / (0*60 +  4.528)
print "opt6 (avoid aflags) ",   (0*60 + 14.609) / (0*60 +  4.528)
print "opt7 (fold_lea)     ",   (0*60 +  9.821) / (0*60 +  4.528)
print

print "speedups from previous:"
print "opt2 (inline, noopt)",   1 / ((1*60 + 36.334) / (9*60 + 20.607))
print "opt3 (coalesce)     ",   1 / ((0*60 + 45.683) / (1*60 + 36.334))
print "opt3 (tls)          ",   1 / ((0*60 + 45.371) / (0*60 + 45.683))
print "opt5 (rle+dse)      ",   1 / ((0*60 + 21.141) / (0*60 + 45.371))
print "opt6 (avoid aflags) ",   1 / ((0*60 + 14.609) / (0*60 + 21.141))
print "opt7 (fold_lea)     ",   1 / ((0*60 +  9.821) / (0*60 + 14.609))
print

print "final speedup:"
print "opt7 (fold_lea)     ",   1 / ((0*60 +  9.821) / (9*60 + 20.607))
