class MapPreprocessor():

    def processMap(map_file_name):
        #produces a black and white pgm file
    
        from matplotlib import pyplot
        import cv2

        THRESHOLD = 127
        MAX = 255

        map_pixel_array = cv2.imread(map_file_name, cv2.IMREAD_GRAYSCALE)

        map_pixel_array_processed = cv2.threshold(map_pixel_array, THRESHOLD, MAX, cv2.THRESH_BINARY)[1]
            
        # pyplot.imshow(map_pixel_array_processed, cmap = 'gray')

        # pyplot.show() # for viewing output pgm file​
        map_file_name_processed = "../maps/fta_f110/f110-map.pgm"
        cv2.imwrite(map_file_name_processed, map_pixel_array_processed)

if ( __name__ == '__main__' ):
    map_file_name = "f110-map.pgm"
    MapPreprocessor.processMap(map_file_name)