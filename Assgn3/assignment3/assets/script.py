import matplotlib.pyplot as plt
import numpy as np
weights = open('ModelWrst_zeros.attach','r').read().splitlines()
W = np.zeros((len(weights),19))
for i,ws in enumerate(weights):
    for j,w in enumerate(ws.split()):
        W[i,j] = float(w)
        
verticesF = open('ModelWrst.obj','r').read().splitlines()
vertexList = []
for vs in verticesF:
    if vs[0] == 'v':
        vertexList.append(map(float,vs.split()[1:]))
        
verts = np.array(vertexList)


skelF = open('ModelWrst.skel','r').read().splitlines()
skelN = np.zeros((len(skelF),3))
for i,skL in enumerate(skelF):
    prnt = int(skL.split()[3])
    if prnt >= 0:
        skelN[i,:] = np.array(map(float,skL.split()[:3])) + skelN[prnt,:]
    else:
        skelN[i,:] = np.array(map(float,skL.split()[:3]))


#plt.figure(1)
#plt.subplot(211)        
#plt.plot(verts[W[:,16]>0,0],verts[W[:,16]>0,1],'*')
#plt.plot(skelN[16:19,0],skelN[16:19,1],'ro')
##plt.show()
#plt.subplot(212)
#plt.plot(verts[W[:,16]>0,0],verts[W[:,16]>0,2],'*')
#plt.plot(skelN[16:19,0],skelN[16:19,2],'ro')
#plt.title('Right hand diagram, first x-y then x-z planes')
#
#plt.figure(2)
#plt.subplot(211)        
#plt.plot(verts[W[:,13]>0,0],verts[W[:,13]>0,1],'*')
#plt.plot(skelN[[13,14,19],0],skelN[[13,14,19],1],'ro')
##plt.show()
#plt.subplot(212)
#plt.plot(verts[W[:,13]>0,0],verts[W[:,13]>0,2],'*')
#plt.plot(skelN[[13,14,19],0],skelN[[13,14,19],2],'ro')
#plt.title('Left hand diagram, first x-y then x-z planes')
#plt.show()

#0.746371, 0.560737, 0.430252
#0.249355, 0.556456, 0.582975
armId = 13
wid = 18;
zm_v = verts[W[:,armId]>0] - skelN[armId+1,:]
zm_nj = skelN[wid+1,:] - skelN[armId+1,:]
dotp = np.dot(zm_v,zm_nj)/(np.linalg.norm(zm_nj)**2)
thresh = 0.1
#plt.plot(zm_v[dotp<thresh,0],zm_v[dotp<thresh,1],'g+')
#plt.plot(zm_v[dotp>=thresh,0],zm_v[dotp>=thresh,1],'b*')
#plt.plot(zm_nj[0],zm_nj[1],'ro')
#plt.plot(np.sign(dotp)*np.linalg.norm(zm_v,axis=1),W[W[:,16]>0,16],'r*')
#plt.show()

armVsIdx = np.where(W[:,armId]>0)[0]
wristAbsVsIdx = armVsIdx[dotp > (1.0+thresh)]
wristProxVsIdx = armVsIdx[np.abs(dotp-1.0) <= thresh]

#plt.plot(verts[wristAbsVsIdx,0],verts[wristAbsVsIdx,1],'b*')
#plt.plot(verts[wristProxVsIdx,0],verts[wristProxVsIdx,1],'g+')
#plt.plot(skelN[16:19,0],skelN[16:19,1],'ro')
#plt.show()

for d,v in zip(dotp[np.abs(dotp-1.0) <= thresh],wristProxVsIdx):
    W[v,wid] = 1.0 / (1.0 + np.exp(-20*(d-1.0)))
    W[v,armId] = 1.0 - W[v,wid]

for v in wristAbsVsIdx:
    W[v,wid] = 1.0
    W[v,armId] = 1.0 - W[v,wid]

f = open('ModelWrst.attach','w')

for i in xrange(W.shape[0]):
    for j in xrange(W.shape[1]):
        f.write('   %.7e'%(W[i,j]))
    f.write('\n')
f.close()	