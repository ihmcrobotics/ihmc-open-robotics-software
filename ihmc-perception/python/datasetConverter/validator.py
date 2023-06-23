from PIL import Image, ImageDraw

class Validator:

    def __init__(self, path, fileName) -> None:
        image_filename = path + "images/" + fileName + ".jpg"
        label_filename = path + "labels/" + fileName + ".txt"
        bboxes = []
        img = Image.open(image_filename)

        with open(label_filename, 'r', encoding='utf8') as f:
            for line in f:
                data = line.strip().split(' ')
                bbox = [float(x) for x in data[1:]]
                bboxes.append(self.yolo_to_xml_bbox(bbox, img.width, img.height))

        self.draw_image(img, bboxes)


    def yolo_to_xml_bbox(self, bbox, w, h):
        # x_center, y_center width heigth
        w_half_len = (bbox[2] * w) / 2
        h_half_len = (bbox[3] * h) / 2
        xmin = int((bbox[0] * w) - w_half_len)
        ymin = int((bbox[1] * h) - h_half_len)
        xmax = int((bbox[0] * w) + w_half_len)
        ymax = int((bbox[1] * h) + h_half_len)
        return [xmin, ymin, xmax, ymax]


    def draw_image(self, img, bboxes):
        draw = ImageDraw.Draw(img)
        for bbox in bboxes:
            draw.rectangle(bbox, outline="red", width=2)
        #img.save("example.jpg")
        img.show()


    # image_filename = "images/medical_pills.jpg"
    # label_filename = "labels/medical_pills.txt"
    # bboxes = []

    # img = Image.open(image_filename)

    # with open(label_filename, 'r', encoding='utf8') as f:
    #     for line in f:
    #         data = line.strip().split(' ')
    #         bbox = [float(x) for x in data[1:]]
    #         bboxes.append(yolo_to_xml_bbox(bbox, img.width, img.height))
