
pub fn load_image_from_path(filepath: &str) -> image::DynamicImage {
        let img = image::open(filepath)
            .expect("Failed to open image");
        return img;
    }