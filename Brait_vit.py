#Fonction calcul brait propre
def calc_vit_brait(list_ind,list_dist,list_state):
    noDetectionDist=0.35
    maxDetectionDist=0.15
    detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] #why are all the values zero, what does this do?
    braitenbergL=[0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8, 0,0,0,0,0,0,0,0] # why are the values like this, how is it calculated
    braitenbergL=[-2*elem for elem in braitenbergL]
    braitenbergR=[0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1, 0,0,0,0,0,0,0,0]
    braitenbergR=[-2*elem for elem in braitenbergR]
    v0=2# what is v0 and why is it set to 2, does it mean a velocity of 2m/s or something
    for i in range(len(list_ind)):
        dist=list_dist[i]
        if(list_state[i] and dist<noDetectionDist):
            print('un objet', str(list_ind[i]))
            if (dist<maxDetectionDist) :
                dist=maxDetectionDist
            detect[list_ind[i]]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
    vLeft=v0
    vRight=v0    
    for i in range(0,16):
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]            
    return vLeft,vRight