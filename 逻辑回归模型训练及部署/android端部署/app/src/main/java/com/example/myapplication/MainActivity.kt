package com.example.myapplication

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.app.AlertDialog
import android.content.DialogInterface
import android.content.res.AssetManager
import android.widget.Button
import android.widget.EditText
import org.tensorflow.lite.Interpreter
import java.io.FileInputStream
import java.lang.Exception
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel

class MainActivity : AppCompatActivity() {

    private lateinit var tflite : Interpreter //声明解释器
    private lateinit var tflitemodel : ByteBuffer //声明模型
    private lateinit var txtValue : EditText
    private lateinit var txtValue1 : EditText
    private lateinit var txtValue2 : EditText
    private lateinit var txtValue3 : EditText
    private lateinit var txtValue4 : EditText
    private lateinit var txtValue5 : EditText
    private lateinit var txtValue6 : EditText
    private lateinit var txtValue7 : EditText
    private lateinit var txtValue8 : EditText

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        txtValue = findViewById<EditText>(R.id.txtValue)
        txtValue1 = findViewById<EditText>(R.id.txtValue1)
        txtValue2 = findViewById<EditText>(R.id.txtValue2)
        txtValue3 = findViewById<EditText>(R.id.txtValue3)
        txtValue4 = findViewById<EditText>(R.id.txtValue4)
        txtValue5 = findViewById<EditText>(R.id.txtValue5)
        txtValue6 = findViewById<EditText>(R.id.txtValue6)
        txtValue7 = findViewById<EditText>(R.id.txtValue7)
        txtValue8 = findViewById<EditText>(R.id.txtValue8)


        // 加载模型
        try{
            tflitemodel = loadModelFile(this.assets, "lr_model.tflite")
            tflite = Interpreter(tflitemodel)
        } catch(ex: Exception){
            ex.printStackTrace()
        }

        var convertButton: Button = findViewById<Button>(R.id.convertButton)
        convertButton.setOnClickListener{
            doInference()
        }
    }

    private fun loadModelFile(assetManager: AssetManager, modelPath: String): ByteBuffer {
        val fileDescriptor = assetManager.openFd(modelPath)
        val inputStream = FileInputStream(fileDescriptor.fileDescriptor)
        val fileChannel = inputStream.channel
        val startOffset = fileDescriptor.startOffset
        val declaredLength = fileDescriptor.declaredLength
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength)
    }

    private fun doInference(){
        var userVal: Float = txtValue.text.toString().toFloat()
        var userVal1: Float = txtValue1.text.toString().toFloat()
        var userVal2: Float = txtValue2.text.toString().toFloat()
        var userVal3: Float = txtValue3.text.toString().toFloat()
        var userVal4: Float = txtValue4.text.toString().toFloat()
        var userVal5: Float = txtValue5.text.toString().toFloat()
        var userVal6: Float = txtValue6.text.toString().toFloat()
        var userVal7: Float = txtValue7.text.toString().toFloat()
        var userVal8: Float = txtValue8.text.toString().toFloat()

        var inputVal: FloatArray = floatArrayOf(userVal, userVal1, userVal2, userVal3, userVal4, userVal5, userVal6, userVal7, userVal8)

        var outputVal: ByteBuffer = ByteBuffer.allocateDirect(4)

        // 检查输入数据 inputval的类型
        println("InputVal data type: ${inputVal::class.simpleName}")
        println("InputVal array: ${inputVal.contentToString()}")


        outputVal.order(ByteOrder.nativeOrder())
        tflite.run(inputVal, outputVal)
        outputVal.rewind()
        var f:Float = outputVal.getFloat()
        val builder = AlertDialog.Builder(this)

        with(builder)
        {
            setTitle("TFLite Interpreter Result")
            val message = if (f < 0.998204) {
                "Your Value is low: $f\n感染风险低"
            } else {
                "Your Value is high: $f\n感染风险高"
            }

            setMessage(message)

            setNeutralButton("OK", DialogInterface.OnClickListener {
                    dialog, id -> dialog.cancel()
            })
            show()
        }

    }

}