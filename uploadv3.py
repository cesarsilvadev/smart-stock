print('Conectando ao S3...')
s3 = boto3.resource(
    service_name = SERVICE_NAME,
    region_name = REGION_NAME,
    aws_access_key_id = AWS_ACCESS_KEY_ID,
    aws_secret_access_key = AWS_SECRET_ACCESS_KEY
)
print('Conectado.')

bucket = s3.Bucket(bucket_name)

for image in images_list:
    bucket.upload_file(str(orders_dir+image),
            f'not_processed/{sending_order}/{os.path.basename(image)}')