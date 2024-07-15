from PIL import Image

# .tifファイルを開く
tif_image = Image.open("/home/aichi2204/Documents/raster.tif")

# .png形式で保存
tif_image.save("/home/aichi2204/Documents/raster.png", "PNG")