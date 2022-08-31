import imageio as io
import pathlib
from os import listdir,mkdir
from os.path import isfile,exists,join
import imgaug.augmenters as ia
from imgaug.augmentables import BoundingBox, BoundingBoxesOnImage,batches
import xml.etree.ElementTree as ET


# Importar diretório das imagens e annotations do dataset:
dir = [join("c://","insper","coisos","teste"),join("c://","Users",'dbkat','Downloads',"aaaa.v1i.voc","train")]
out = join("c://","insper",'coisos','out')
if not (exists(join(out,'obj'))):
    mkdir(join(out,"obj"))


# Importar nome das labels:
clsid = {"arroz":0,"nada":1}

def get_boxes(xml):
    boxes = list()
    tree = ET.parse(xml)
    root = tree.getroot()
    for obj in root.iter("object"):
        cls = obj.find('name').text
        xmlbox = obj.find('bndbox')
        b = (float(xmlbox.find('xmin').text), float(xmlbox.find('xmax').text), float(xmlbox.find('ymin').text), float(xmlbox.find('ymax').text))
        box_data = BoundingBox(x1=b[0],y1=b[2],x2=b[1],y2=b[3],label=cls)
        boxes.append(box_data)
    return boxes

def get_size(xml):
    tree = ET.parse(xml)
    root = tree.getroot()
    size = root.find("size")
    w = int(size.find('width').text)
    h = int(size.find('height').text)
    size = [w,h]
    return size

def convert(xmin,xmax,ymin,ymax,size):
    dw = 1./(size[0])
    dh = 1./(size[1])
    x = (xmin + xmax)/2.0 - 1
    y = (ymin + ymax)/2.0 - 1
    w = xmax - xmin
    h = ymax - ymin
    x = x*dw
    w = w*dw
    y = y*dh
    h = h*dh
    return (x,y,w,h)

def convert_annotation(bbs,size):
    bimgs=list()
    for j in range(len(bbs)):
        bimg = list()
        for bb in bbs[j].items:
            xmin = bb.x1
            xmax = bb.x2
            ymin = bb.y1
            ymax = bb.y2
            cl = bb.label
            conv = convert(xmin,xmax,ymin,ymax,size[j])
            bimg.append([conv,cl])
        bimgs.append(bimg)
    return bimgs 


seq = ia.Sequential([
    ia.GaussianBlur(sigma=(0,4)),
    ia.Affine(rotate=(-25,25),scale=(0.8,1.2)),
    ia.MultiplyAndAddToBrightness(add=(-25,25))
],random_order=True)

for label_dir in listdir(dir[0]):
    # Coletar informações dos diretórios
    path_cont = listdir(join(dir[0],label_dir))
    path_cont2 = listdir(join(dir[1],label_dir))
    img_names = list()
    img_list = list()
    img_shape = list()
    xml_list = list()
    size_list = list()
    for k in range(len(path_cont)):
        img_names.append(path_cont[k])
        img_list.append(io.imread(join(dir[0],label_dir,path_cont[k])))
        img_shape.append(tuple(img_list[k].shape))
        io.imwrite(join(out,"obj")+"/"+img_names[k],img_list[k])
    for k in range(len(path_cont2)):
        xml_list.append(join(dir[1],label_dir,path_cont2[k]))
        size_list.append(get_size(xml_list[k]))
    bbs_list = [BoundingBoxesOnImage(get_boxes(xml_list[i]),shape=tuple(img_shape[i])) for i in range(len(img_shape))]

    # Preparar criação de novos arquivos e pastas
    annot_original = convert_annotation(bbs_list,size_list)
    batch = batches.Batch(images=img_list,bounding_boxes=bbs_list)
    batch_aug = seq.augment_batch_(batch=batch)
    aug_imgs = batch_aug.images_aug
    path0 = join(out,"obj")
    
    # Conversão das annotations originais
    txt = str()
    for a in range(len(annot_original)):
        with open(path0+"/"+img_names[a]+".txt","w") as f2:
            for b,cla in annot_original[a]:
                f2.write(("{0} {1} {2} {3} {4}".format(clsid[cla],b[0],b[1],b[2],b[3]))+"\n")
            txt += ("data/obj/" + img_names[a] + "\n")
    with open(out + "/train.txt",'w') as tr:
        tr.write(txt)
    
    # Formar novas imagens e annotations
    for i in range(len(aug_imgs)):
        io.imwrite(str(path0)+'/'+"aug_"+(img_names[i]),aug_imgs[i])
    bbs_aug = batch_aug.bounding_boxes_aug
    yolo = convert_annotation(bbs_aug,size_list)
    txt2 = str() 
    for c in range(len(yolo)):
        with open(path0+"/"+"aug_"+img_names[c]+".txt","w") as f2:
            for b,cla in yolo[c]:
                f2.write(("{0} {1} {2} {3} {4}".format(clsid[cla],b[0],b[1],b[2],b[3]))+"\n")
            txt2 += ("data/obj/" + "aug_" + img_names[c] + "\n")
    with open(out+"/train.txt",'a') as tr:
        tr.write(txt2)

# Criando arquivo das labels:
with open(out+"/obj.names",'w') as f3:
    for cls in clsid:
        f3.write(cls + "\n")

# Criando arquivo .data:
with open(out+"/obj.data",'w') as dat:
    dat.write(("classes = {0}".format(len(clsid)))+"\n")
    dat.write("train = data/train.txt" + '\n')
    dat.write('names = data/obj.names' + '\n')
    dat.write('backup = Hazmat_identification/' + '\n') 
print("Processo realizado com sucesso")