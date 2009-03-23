import pdb
import pygame
from pygame.locals import DOUBLEBUF, MOUSEBUTTONDOWN, QUIT, KEYDOWN,K_ESCAPE
import Box2D as box2d
import sys
from math import sqrt

class fwContactTypes:
    """
    Acts as an enum, holding the types necessary for contacts:
    Added, persisted, and removed
    """
    contactUnknown = 0
    contactAdded = 1
    contactPersisted = 2
    contactRemoved = 3

class fwContactPoint:
    """
    Structure holding the necessary information for a contact point.
    All of the information is copied from the contact listener callbacks.
    """
    shape1 = None
    shape2 = None
    normal = None
    position = None
    velocity = None
    id  = None
    state = 0

        

worldAABB=box2d.b2AABB()
worldAABB.lowerBound = (-40.0, -30.0)
worldAABB.upperBound = ( 40.0, 40.0)
gravity = (0.0, -10.0)
world = box2d.b2World(worldAABB, gravity, True)

bodyDef = box2d.b2BodyDef()
bodyDef.position = (0, -25)
bodyDef.angularDamping = 1000000
bodyDef.fixedRotation = True
body = world.CreateBody(bodyDef)
shapeDef = box2d.b2PolygonDef()
shapeDef.SetAsBox(6, 0.8)
shapeDef.density = 0.1
shapeDef.friction = 0
shapeDef.restitution = 1 
shapeDef.userData = 'bat'
body.CreateShape(shapeDef)
body.SetMassFromShapes()

ballbodydef = box2d.b2BodyDef()
ballbodydef.position = (0,-20)
ballbodydef.isBullet = True
ballbodydef.angularDamping = 1000000
ball = world.CreateBody(ballbodydef)
ballshapedef = box2d.b2CircleDef()
ballshapedef.density = 0.1 
ballshapedef.friction = 0
ballshapedef.restitution = 0.8 
ballshapedef.radius = 2
ballshapedef.userData = 'ball'
ball.CreateShape(ballshapedef)
ball.SetMassFromShapes()

groundBodyDef = box2d.b2BodyDef()
groundBodyDef.position = (0, 0)
groundBody = world.CreateBody(groundBodyDef)
groundShapeDef = box2d.b2PolygonDef()
groundShapeDef.friction =0.3
groundShapeDef.SetAsBox(80, 1, (0,-29),0)
groundShapeDef.userData = "wall"
groundBody.CreateShape(groundShapeDef)
groundShapeDef.SetAsBox(80, 1, (0,29),0)
groundBody.CreateShape(groundShapeDef)
groundShapeDef.SetAsBox(1, 80, (39,0),0)
groundBody.CreateShape(groundShapeDef)
groundShapeDef.SetAsBox(1, 80, (-39,0),0)
groundBody.CreateShape(groundShapeDef)
timeStep = 0.75 / 60.0
velocityIterations = 3 
positionIterations = 3

pygame.init()
screen = pygame.display.set_mode((800, 600),DOUBLEBUF, 32)
clock = pygame.time.Clock()

#blockdef = box2d.b2BodyDef()
#blockdef.position = (0, 0) 
#blockbody = world.CreateBody(groundBodyDef)
#blockShapeDef = box2d.b2PolygonDef()
#blockShapeDef.friction =0.3
#blockShapeDef.SetAsBox(2, 2, (10,15),0)
#blockShapeDef.userData = 'block'
#blockshape = blockbody.CreateShape(blockShapeDef)

def toScreen(pt):
    return (int((pt.x)*10 + 400), int(600-(pt.y)*10 - 300))
def toScreen_t(pt):
    return (int((pt[0])*10 + 400), int(600-(pt[1])*10 - 300))
def toWorld(pt):
    return ((pt[0]-400)/10.0 , (-pt[1]+300)/10.0)
def shape_pos(body,shape):
    return [toScreen(body.GetWorldPoint(localPoint)) for localPoint in shape.vertices]

class Block(object):

    def __init__(self, position, x, y, destroy =0, shape ='box', col = (100,10,60)):

        self.position = position
        self.x = x
        self.y = y
        self.col = col
        self.shape_type = shape
        self.destroy = destroy
        self.blockdef = box2d.b2BodyDef()
        self.blockdef.position = position 
        self.shape()
        self.alive = False 

    def shape(self):

        if self.shape_type == 'box':
            self.blockShapeDef = box2d.b2PolygonDef()
            self.blockShapeDef.friction =0.3
            self.blockShapeDef.SetAsBox(self.x, self.y)#, self.position ,0)
        if self.shape_type == 'circle':
            self.blockShapeDef = box2d.b2CircleDef()
            self.blockShapeDef.friction =0.3
            #self.blockShapeDef.localPosition.Set(*self.position)
            self.blockShapeDef.localPosition.Set(0,0)
            self.blockShapeDef.radius = self.x
        if self.shape_type == 'triangle':
            self.blockShapeDef = box2d.b2PolygonDef()
            self.blockShapeDef.friction =0.3
            self.blockShapeDef.setVertices( [ (-self.x, 0.0), (self.x, 0.0), (0.0, self.y*2) ])
                                                  


    def add_to_blocks(self,blocks):
        
        self.blockShapeDef.userData = 'block%s' % blocks.number
        self.blockbody = world.CreateBody(self.blockdef)
        self.alive = True 
        self.blockshape = self.blockbody.CreateShape(self.blockShapeDef)
        blocks.blockbodies[self.blockShapeDef.userData] = self
        blocks.number = blocks.number + 1
        self.blocks = blocks

    def draw(self):
        if self.alive is False:
            return
        if self.shape_type in ('box','triangle'):
            pygame.draw.polygon(screen, self.col, shape_pos(self.blockbody, self.blockbody.shapeList[0]), 3)
        if self.shape_type == 'circle':
            pygame.draw.circle(screen, self.col, toScreen_t(self.position), self.x*10, 2)

    def delete(self, point):

        if self.destroy == 0:
            self.blocks.to_delete.append(self.blockShapeDef.userData)
        elif self.destroy > 0:
            if (ball.linearVelocity.x * point.normal.x) ** 2 + (ball.linearVelocity.y * point.normal.y) ** 2 > self.destroy:
                self.blocks.to_delete.append(self.blockShapeDef.userData)
        else:
            if (ball.linearVelocity.x * point.normal.x) ** 2 + (ball.linearVelocity.y * point.normal.y) ** 2 < abs(self.destroy):
                self.blocks.to_delete.append(self.blockShapeDef.userData)


class Blocks(object):
    
    def __init__(self):
        self.blockbodies = {}
        self.number = 1
        self.to_delete = []

    def add_block(self, block):

        block.add_to_blocks(self)

    def delete_blocks(self):
        for block in self.to_delete:
            if self.blockbodies[block].alive == True:
                self.blockbodies[block].blockbody.ClearUserData()
                world.DestroyBody(self.blockbodies[block].blockbody)
                self.blockbodies[block].alive = False
        to_delete = []

    def draw_blocks(self):
        for block in self.blockbodies.itervalues():
            block.draw()

    

blocks = Blocks()
for l in range(-1,3):
    for i in range(-4,3):
        blocks.add_block(Block((i*8,l*8),2,2, 800, 'circle'))





md = box2d.b2MouseJointDef()
md.body1   = body # world.GetGroundBody()
md.body2   = body
md.target  = body.GetWorldCenter() # toWorld(pygame.mouse.get_pos())
md.maxForce= 100000.0 
mouseJoint = world.CreateJoint(md).getAsType()
body.WakeUp()
delete = []

class fwContactListener(box2d.b2ContactListener):
    """
    Handles all of the contact states passed in from Box2D.

    """
    test = None
    def __init__(self):
        super(fwContactListener, self).__init__()

    def handleCall(self, state, point):
        if not self.test: return

        cp          = fwContactPoint()
        cp.shape1   = point.shape1
        cp.shape2   = point.shape2
        cp.position = point.position.copy()
        cp.normal   = point.normal.copy()
        cp.id       = point.id
        cp.state    = state
        self.test.points.append(cp)

    def Add(self, point):
        pass

    def Persist(self, point):
        pass

    def Remove(self, point):
        if (point.shape1.userData == 'ball' and point.shape2.userData == "bat" ) or\
          (point.shape2.userData == 'ball' and point.shape1.userData == "bat" ):

            new = body.linearVelocity.copy()
            new.y = abs(new.y) 
            if new.x >= 0:
                new.x = -sqrt(abs(new.x*1.5))
            else:
                new.x = sqrt(abs(new.x*1.5))

            ball.ApplyImpulse(new,ball.GetWorldCenter())

        if (point.shape1.userData == 'ball' and point.shape2.userData.startswith("block") ) or\
          (point.shape2.userData == 'ball' and point.shape1.userData.startswith("block") ):

            if point.shape2.userData.startswith("block"):
                userdata = point.shape2.userData
            else:
                userdata = point.shape1.userData


             
            global blocks
            blocks.blockbodies[userdata].delete(point)
            #if (ball.linearVelocity.x - point.normal.x) ** 2 + (ball.linearVelocity.y - point.normal.y) ** 2 < 650:
            #    blocks.to_delete.append(userdata)


start = True
freemouse = False

contact_listener = fwContactListener()
world.SetContactListener(contact_listener)
loop =True
while loop:
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            loop =False
        if event.type == KEYDOWN:
            if freemouse == False:
                freemouse = True
            else:
                freemouse = False 

    world.Step(timeStep, velocityIterations, positionIterations)

    if start:
        pygame.mouse.set_pos(400,570)
        mouse_pos = (400,570)
        start = False
    else:
        mouse_pos = pygame.mouse.get_pos()
        if mouse_pos[0] > 780 :
            mouse_pos = (780, mouse_pos[1])
        if mouse_pos[0] < 20 :
            mouse_pos = (20, mouse_pos[1])
        if mouse_pos[1] > 580 :
            mouse_pos = (mouse_pos[0], 580)
        if mouse_pos[1] < 450 :
            mouse_pos = (mouse_pos[0], 450)
        if not freemouse:
            pygame.mouse.set_pos(mouse_pos)


    if not freemouse:
        mouseJoint.SetTarget(toWorld(mouse_pos))
    screen.fill( (0,0,0) )
    pygame.draw.circle(screen, (100,10,60), toScreen(ball.position) , 20, 2)
    #pygame.draw.rect(screen,(100,10,60), pygame.Rect(pos[0] -10 , pos[1] -10,20,20), 3)
    pygame.draw.polygon(screen, (100,10,60), shape_pos(body, body.shapeList[0]), 3)
#    if delete and blockbody:
#        world.DestroyBody(blockbody)
#        blockbody = None
#    if blockbody:
 #       pygame.draw.polygon(screen, (100,10,60), shape_pos(blockbody, blockbody.shapeList[0]), 3)

    blocks.delete_blocks()
    blocks.draw_blocks()
    #pygame.draw.rect(screen,(100,10,60), pygame.Rect(pos2[0] -10 , pos2[1] -10,20,20), 3)
    #pygame.draw.polygon(screen, (100,10,60), shape_pos(body2, body2.shapeList[0]), 3)
    pygame.display.flip()
    #pos = [p + 1 for p in pos]

sys.exit(0)


