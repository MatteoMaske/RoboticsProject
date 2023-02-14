## Import all relevant libraries
import bpy
import numpy as np
import math as m
import random
import time
#self.obj_names = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']

blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER',
'X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1',
'X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']
blocksDict = {'X1-Y1-Z2': 0, 'X1-Y2-Z1': 1, 'X1-Y2-Z2': 2, 'X1-Y2-Z2-CHAMFER': 3, 
'X1-Y2-Z2-TWINFILLET': 4, 'X1-Y3-Z2': 5, 'X1-Y3-Z2-FILLET': 6, 'X1-Y4-Z1': 7,
 'X1-Y4-Z2': 8, 'X2-Y2-Z2': 9, 'X2-Y2-Z2-FILLET': 10}
DEBUG = False

colors = [(0.0, 0.0, 0.0, 1.0),
    (0.0, 0.0, 1.0, 1.0),
    (0.0, 1.0, 0.0, 1.0),
    (0.0, 1.0, 1.0, 1.0),
    (1.0, 0.0, 0.0, 1.0),
    (1.0, 0.0, 1.0, 1.0),
    (1.0, 1.0, 0.0, 1.0), 
    (1.0, 1.0, 1.0, 1.0)]

## Blender variables
# Define the scene information
BLOCK_SPAWN_Z = 0.367
BLOCK_SPAWN_Z1 = 0.37
BLOCK_SPAWN_X_BOUNDS = [-0.2, 0.2]
BLOCK_SPAWN_Y_BOUNDS = [0.08, 0.29]
## Main Class
class Render:
    def __init__(self):
        ## Scene information
        # Define the scene information
        self.scene = bpy.data.scenes['Scene']
        # Define the information relevant to the <bpy.data.objects>
        self.camera = bpy.data.objects['Camera']
        self.light_1 = bpy.data.objects['Light1']
        self.light_2 = bpy.data.objects['Light2']
        self.objects = []

        ## Output information
        # Input your own preferred location for the images and labels
        self.images_filepath = f'/Users/amirgheser/Robotics/dataset/'
        self.labels_filepath = f'/Users/amirgheser/Robotics/dataset/labels'

    def main_rendering_loop(self):
        '''
        This function represent the main algorithm explained in the Tutorial, it accepts the
        rotation step as input, and outputs the images and the labels to the above specified locations.
        '''

        if (DEBUG):
            self.samples = 10
        else:
            print("How many samples do you want to render?")
            self.samples = int(input()) # Ask the user for the number of samples to render
            accept_render = input('\nContinue?[Y/N]:  ') # Ask whether to procede with the data generation

        if DEBUG or accept_render == 'Y': # If the user inputs 'Y' then procede with the data generation
            # Create .txt file that record the progress of the data generation
            report_file_path = self.labels_filepath + '/progress_report.txt'
            report = open(report_file_path, 'w+')
            # Multiply the limits by 10 to adapt to the for loop

            # Define a counter to name each .png and .txt files that are outputted
            render_counter = 0
            # Define the step with which the pictures are going to be taken
            # Create material to change the color of the object
            random.seed(random.randint(1, 1000))
            # Begin nested loops
            ###############################
            optionsX = np.arange(BLOCK_SPAWN_X_BOUNDS[0]+1, BLOCK_SPAWN_X_BOUNDS[1]+1, 0.1).tolist()
            optionsX = [x-1 for x in optionsX]
            optionsY = np.arange(BLOCK_SPAWN_Y_BOUNDS[0], BLOCK_SPAWN_Y_BOUNDS[1], 0.1).tolist()
            # options = cartesian product of optionsX and optionsY
            options = [(x, y) for x in optionsX for y in optionsY]
            for _ in range(self.samples):
                number_of_blocks = random.randint(1, 6)
                print("Number of blocks: ", number_of_blocks)
                blockCoords = random.sample(options, k=number_of_blocks)
                for nth_block in range(number_of_blocks):
                    mat = bpy.data.materials.new(name= 'colored')
                    ### Import random blocks in random positions
                    # Select block
                    incoming_block_name = random.choice(blocks)
                    incoming_block = bpy.data.objects[incoming_block_name]
                    self.objects.append(incoming_block)
                    # Set the imported object's location
                    incoming_block.location = (blockCoords[nth_block][0],
                                            blockCoords[nth_block][1],
                                    BLOCK_SPAWN_Z1 if incoming_block_name[7] == '1' else BLOCK_SPAWN_Z)


                    # Set the imported object's rotation
                    incoming_block.rotation_euler = (0, ## change here to get different rotations
                                        0, ## change here to get different rotations
                                        random.uniform(0, 2*m.pi))
                    # Change the imported object's color
                    incoming_block.active_material = mat
                    mat.diffuse_color = (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), 1.0)
                    # Assign the material to the block
                render_counter += 1 # Update counter
                ## Configure lighting
                energy1 = random.randint(0, 30) # Grab random light intensity
                self.light_1.data.energy = energy1 # Update the <bpy.data.objects['Light']> energy information
                energy2 = random.randint(4, 20) # Grab random light intensity
                self.light_2.data.energy = energy2 # Update the <bpy.data.objects['Light2']> energy information

                ## Generate render
                self.render_blender(render_counter) # Take photo of current scene and ouput the render_counter.png file
                # Display demo information - Photo information
                print("--> Picture information:")
                print("     Resolution:", (self.xpix*self.percentage, self.ypix*self.percentage))
                print("     Rendering samples:", self.samples)

                ## Output Labels
                text_file_name = self.labels_filepath + '/' + str(render_counter) + '.txt' # Create label file name
                text_file = open(text_file_name, 'w+') # Open .txt file of the label
                # Get formatted coordinates of the bounding boxes of all the objects in the scene
                # Display demo information - Label construction
                print("---> Label Construction")
                text_coordinates = self.get_all_coordinates()
                splitted_coordinates = text_coordinates.split('\n')[:-1] # Delete last '\n' in coordinates
                text_file.write('\n'.join(splitted_coordinates)) # Write the coordinates to the text file and output the render_counter.txt file
                text_file.close() # Close the .txt file corresponding to the label

                ## Remove all blocks
                for obj in blocks:
                   bpy.data.objects[obj].location.z = 100
                self.objects = []

                ## Show progress on batch of renders
                if render_counter % 100 == 0:
                    print('Progress =', str(render_counter) + '/' + str(self.samples))
                report.write('Progress: ' + str(render_counter) + '\n')

            report.close() # Close the .txt file corresponding to the report

        else: # If the user inputs anything else, then abort the data generation
            print('Aborted rendering operation')
            pass

    def get_all_coordinates(self):
        '''
        This function takes no input and outputs the complete string with the coordinates
        of all the objects in view in the current image
        '''
        main_text_coordinates = '' # Initialize the variable where we'll store the coordinates
        for i, objct in enumerate(self.objects): # Loop through all of the objects
            print("     On object:", objct)
            b_box = self.find_bounding_box(objct) # Get current object's coordinates
            if b_box: # If find_bounding_box() doesn't return None
                print("         Initial coordinates:", b_box)
                text_coordinates = self.format_coordinates(b_box, objct) # Reformat coordinates to YOLOv3 format
                print("         YOLO-friendly coordinates:", text_coordinates)
                main_text_coordinates = main_text_coordinates + text_coordinates # Update main_text_coordinates variables whith each
                                                                                 # line corresponding to each class in the frame of the current image
            else:
                #print("         Object not visible")
                pass

        return main_text_coordinates # Return all coordinates

    def format_coordinates(self, coordinates, objct):
        '''
        This function takes as inputs the coordinates created by the find_bounding box() function, the current class,
        the image width and the image height and outputs the coordinates of the bounding box of the current class
        '''
        classe = blocksDict[objct.name]
        # If the current class is in view of the camera
        if coordinates: 
            ## Change coordinates reference frame
            x1 = (coordinates[0][0])
            x2 = (coordinates[1][0])
            y1 = (1 - coordinates[1][1])
            y2 = (1 - coordinates[0][1])

            ## Get final bounding box information
            width = (x2-x1)  # Calculate the absolute width of the bounding box
            height = (y2-y1) # Calculate the absolute height of the bounding box
            # Calculate the absolute center of the bounding box
            cx = x1 + (width/2) 
            cy = y1 + (height/2)

            ## Formulate line corresponding to the bounding box of one class
            txt_coordinates = str(classe) + ' ' + str(cx) + ' ' + str(cy) + ' ' + str(width) + ' ' + str(height) + '\n'

            return txt_coordinates
        # If the current class isn't in view of the camera, then pass
        else:
            pass

    def find_bounding_box(self, obj):
        """
        Returns camera space bounding box of the mesh object.
        Gets the camera frame bounding box, which by default is returned without any transformations applied.
        Create a new mesh object based on self.carre_bleu and undo any transformations so that it is in the same space as the
        camera frame. Find the min/max vertex coordinates of the mesh visible in the frame, or None if the mesh is not in view.
        :param scene:
        :param camera_object:
        :param mesh_object:
        :return:
        """

        """ Get the inverse transformation matrix. """
        matrix = self.camera.matrix_world.normalized().inverted()
        """ Create a new mesh data block, using the inverse transform matrix to undo any transformations. """
        mesh = obj.to_mesh(preserve_all_data_layers=True)
        mesh.transform(obj.matrix_world)
        mesh.transform(matrix)

        """ Get the world coordinates for the camera frame bounding box, before any transformations. """
        frame = [-v for v in self.camera.data.view_frame(scene=self.scene)[:3]]

        lx = []
        ly = []

        for v in mesh.vertices:
            co_local = v.co
            z = -co_local.z

            if z <= 0.0:
                """ Vertex is behind the camera; ignore it. """
                continue
            else:
                """ Perspective division """
                frame = [(v / (v.z / z)) for v in frame]

            min_x, max_x = frame[1].x, frame[2].x
            min_y, max_y = frame[0].y, frame[1].y

            x = (co_local.x - min_x) / (max_x - min_x)
            y = (co_local.y - min_y) / (max_y - min_y)

            lx.append(x)
            ly.append(y)


        """ Image is not in view if all the mesh verts were ignored """
        if not lx or not ly:
            return None

        min_x = np.clip(min(lx), 0.0, 1.0)
        min_y = np.clip(min(ly), 0.0, 1.0)
        max_x = np.clip(max(lx), 0.0, 1.0)
        max_y = np.clip(max(ly), 0.0, 1.0)

        """ Image is not in view if both bounding points exist on the same side """
        if min_x == max_x or min_y == max_y:
            return None

        """ Figure out the rendered image size """
        render = self.scene.render
        fac = render.resolution_percentage * 0.01
        dim_x = render.resolution_x * fac
        dim_y = render.resolution_y * fac
        
        ## Verify there's no coordinates equal to zero
        coord_list = [min_x, min_y, max_x, max_y]
        if min(coord_list) == 0.0:
            indexmin = coord_list.index(min(coord_list))
            coord_list[indexmin] = coord_list[indexmin] + 0.0000001

        return (min_x, min_y), (max_x, max_y)

    def render_blender(self, count_f_name):
        # Define random parameters
        random.seed(random.randint(1,1000))
        self.xpix = 1280/100
        self.ypix = 720/100
        self.percentage = 100
        self.samples = 25
        # Render images
        image_name = str(count_f_name) + '.png'
        self.export_render(self.xpix, self.ypix, self.percentage, self.samples, self.images_filepath, image_name)

    def export_render(self, res_x, res_y, res_per, samples, file_path, file_name):
        # Set all scene parameters
        bpy.context.scene.cycles.samples = samples
        self.scene.render.resolution_x = int(res_x * res_per)
        self.scene.render.resolution_y = int(res_y * res_per)
        self.scene.render.resolution_percentage = res_per
        self.scene.render.filepath =  file_path + '/' + file_name

        # Take picture of current visible scene
        bpy.ops.render.render(write_still=True)

    def create_objects(self):  # This function creates a list of all the <bpy.data.objects>
        objs = []
        for obj in self.obj_names:
            objs.append(bpy.data.objects[obj])

        return objs

## Run data generation
if __name__ == '__main__':
    print("\n\n\nStarting data generation...\n\n\n")
    for block in blocks:
        bpy.data.objects[block].location.x = 100
        bpy.data.objects[block].scale = (0.7, 0.7, 0.7)
    # Initialize rendering class as r
    r = Render()
    # Initialize camera
    # Begin data generation
    r.main_rendering_loop()